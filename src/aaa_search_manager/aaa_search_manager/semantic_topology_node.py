"""Build per-robot and fused semantic topology graphs during exploration."""
from __future__ import annotations

from functools import partial
import json
import math
from pathlib import Path
import os
import tempfile

import rclpy
from aaa_search_interfaces.msg import TargetObservation
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .semantic_topology_core import GraphFusion, RobotGraph
from .target_transform import transform_point_and_covariance


class SemanticTopologyNode(Node):
    def __init__(self):
        super().__init__('semantic_topology')
        defaults = {
            'robot_ids': ['hyzx001', 'jetson003'],
            'base_frames': ['hyzx001/base_footprint', 'jetson003/base_link'],
            'global_frame': 'site_map',
            'observation_topic_prefix': '/search/target_observation',
            'map_topic': '/swarm/map',
            'emergency_stop_topic': '/swarm/emergency_stop',
            'local_graph_topic_prefix': '/semantic_topology/local',
            'fused_graph_topic': '/semantic_topology/fused',
            'output_directory': '/tmp/aaa_semantic_topology',
            'detection_confidence': 0.50, 'merge_distance': 0.20,
            'min_observations': 1, 'observation_interval': 0.5,
            'min_observation_baseline': 0.15, 'max_weight': 10.0,
            'stand_off_distance': 0.30, 'visit_radius': 0.30,
            'leave_radius': 0.45, 'sample_distance': 0.04,
            'sample_yaw_deg': 5.0, 'localization_jump_limit': 0.20,
            'minimum_path_length': 0.30, 'occupied_threshold': 50,
            'robot_safety_radius': 0.15, 'tf_timeout_sec': 0.20,
            'pose_update_period': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        robot_ids = [str(v) for v in self.get_parameter('robot_ids').value]
        frames = [str(v) for v in self.get_parameter('base_frames').value]
        if not robot_ids or len(robot_ids) != len(frames):
            raise ValueError('robot_ids and base_frames must be non-empty parallel lists')
        self.frames = dict(zip(robot_ids, frames))
        self.global_frame = str(self.get_parameter('global_frame').value)
        graph_args = {name: self.get_parameter(name).value for name in (
            'detection_confidence', 'merge_distance', 'min_observations',
            'observation_interval', 'min_observation_baseline', 'max_weight',
            'stand_off_distance', 'visit_radius', 'leave_radius',
            'sample_distance', 'sample_yaw_deg', 'localization_jump_limit',
            'minimum_path_length')}
        self.graphs = {robot_id: RobotGraph(robot_id, **graph_args) for robot_id in robot_ids}
        self.fusion = GraphFusion(float(self.get_parameter('merge_distance').value))
        self.map = None
        self.emergency_stop = False
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.output_directory = Path(str(self.get_parameter('output_directory').value)).expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local_publishers = {
            rid: self.create_publisher(String,
                f"{str(self.get_parameter('local_graph_topic_prefix').value).rstrip('/')}/{rid}", latched)
            for rid in robot_ids
        }
        self.fused_publisher = self.create_publisher(
            String, str(self.get_parameter('fused_graph_topic').value), latched)
        prefix = str(self.get_parameter('observation_topic_prefix').value).rstrip('/')
        self.observation_subscriptions = [
            self.create_subscription(TargetObservation, f'{prefix}/{rid}',
                                     partial(self._on_observation, rid), 10)
            for rid in robot_ids
        ]
        self.create_subscription(OccupancyGrid, str(self.get_parameter('map_topic').value),
                                 self._on_map, latched)
        self.create_subscription(Bool, str(self.get_parameter('emergency_stop_topic').value),
                                 self._on_emergency_stop, latched)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=300.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(float(self.get_parameter('pose_update_period').value), self._update_poses)
        self._publish()

    def _on_map(self, message):
        if (message.header.frame_id == self.global_frame
                and len(message.data) == int(message.info.width) * int(message.info.height)):
            self.map = message

    def _on_emergency_stop(self, message):
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            for graph in self.graphs.values():
                graph.invalidate_segment()

    def _point_is_free(self, point):
        message = self.map
        if message is None or message.info.resolution <= 0.0:
            return False
        q = message.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        dx, dy = point[0] - message.info.origin.position.x, point[1] - message.info.origin.position.y
        c, s = math.cos(yaw), math.sin(yaw)
        col = int(math.floor((c * dx + s * dy) / message.info.resolution))
        row = int(math.floor((-s * dx + c * dy) / message.info.resolution))
        radius = int(math.ceil(float(self.get_parameter('robot_safety_radius').value) / message.info.resolution))
        threshold = int(self.get_parameter('occupied_threshold').value)
        width, height = int(message.info.width), int(message.info.height)
        for rr in range(row-radius, row+radius+1):
            for cc in range(col-radius, col+radius+1):
                if (rr-row)**2 + (cc-col)**2 > radius**2:
                    continue
                if rr < 0 or cc < 0 or rr >= height or cc >= width:
                    return False
                value = int(message.data[rr * width + cc])
                if value < 0 or value >= threshold:
                    return False
        return True

    def _point_in_map(self, point):
        message = self.map
        if message is None or message.info.resolution <= 0.0:
            return False
        q = message.info.origin.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        dx, dy = point[0] - message.info.origin.position.x, point[1] - message.info.origin.position.y
        c, s = math.cos(yaw), math.sin(yaw)
        col = int(math.floor((c*dx + s*dy) / message.info.resolution))
        row = int(math.floor((-s*dx + c*dy) / message.info.resolution))
        return 0 <= col < int(message.info.width) and 0 <= row < int(message.info.height)

    def _lookup(self, target, source, stamp):
        return self.tf_buffer.lookup_transform(
            target, source, stamp, timeout=Duration(seconds=self.tf_timeout))

    def _on_observation(self, expected_robot, message):
        if message.robot_id != expected_robot or not message.target_label.strip():
            return
        source_frame = message.header.frame_id.lstrip('/')
        if source_frame != self.frames[expected_robot]:
            self.get_logger().warning(f'rejected {expected_robot} observation in {source_frame!r}')
            return
        try:
            stamp = Time.from_msg(message.header.stamp)
            transform = self._lookup(self.global_frame, source_frame, stamp)
            t, q = transform.transform.translation, transform.transform.rotation
            point, _ = transform_point_and_covariance(
                (message.position.x, message.position.y, message.position.z),
                message.position_covariance, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            if float(message.valid_depth_ratio) <= 0.0 or not self._point_in_map(point[:2]):
                self.get_logger().warning('dropped semantic observation with invalid depth or map position')
                return
            node = self.graphs[expected_robot].observe(
                message.target_label, (float(point[0]), float(point[1])),
                float(message.semantic_confidence), (float(t.x), float(t.y)),
                stamp.nanoseconds / 1e9, self._point_is_free)
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'dropped semantic observation: {error}')
            return
        if node is not None:
            self._publish()

    def _update_poses(self):
        changed = False
        for robot_id, frame in self.frames.items():
            try:
                transform = self._lookup(self.global_frame, frame, Time())
                t, q = transform.transform.translation, transform.transform.rotation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
                changed |= self.graphs[robot_id].update_pose(
                    t.x, t.y, yaw, emergency_stop=self.emergency_stop) is not None
            except TransformException:
                self.graphs[robot_id].invalidate_segment()
        if changed:
            self._publish()

    @staticmethod
    def _atomic_json(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=str(path.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(value, stream, ensure_ascii=False, indent=2)
                stream.write('\n')
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _publish(self):
        for robot_id, graph in self.graphs.items():
            document = graph.public(confirmed_only=True)
            document['frame_id'] = self.global_frame
            text = json.dumps(document, ensure_ascii=False, separators=(',', ':'))
            self.local_publishers[robot_id].publish(String(data=text))
            self._atomic_json(self.output_directory / f'{robot_id}.json', document)
        fused = self.fusion.update(self.graphs)
        self.fused_publisher.publish(String(data=json.dumps(fused, ensure_ascii=False, separators=(',', ':'))))
        self._atomic_json(self.output_directory / 'fused.json', fused)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticTopologyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
