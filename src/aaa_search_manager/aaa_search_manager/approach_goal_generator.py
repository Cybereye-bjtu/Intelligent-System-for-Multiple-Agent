from __future__ import annotations

import math

import numpy as np
import rclpy
from aaa_search_interfaces.msg import NavigationTask, TargetObservation
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .approach_planner import generate_approach_goal, generate_approach_pair


class ApproachGoalGenerator(Node):
    def __init__(self):
        super().__init__('approach_goal_generator')
        self.declare_parameter('robot_ids', ['hyzx001', 'jetson003'])
        self.declare_parameter(
            'base_frames', ['hyzx001/base_footprint', 'jetson003/base_link']
        )
        self.declare_parameter('global_frame', 'site_map')
        self.declare_parameter('target_topic', '/search/target_observation_global')
        self.declare_parameter('navigation_timeout_sec', 180.0)
        self.declare_parameter('radii', [0.3, 0.5, 0.8])
        self.declare_parameter('angle_samples', 24)
        self.declare_parameter('max_goal_cost', 75)
        self.declare_parameter('lethal_cost', 96)
        self.declare_parameter('min_pair_separation', 0.30)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.robot_ids = [str(value) for value in self.get_parameter('robot_ids').value]
        base_frames = [str(value) for value in self.get_parameter('base_frames').value]
        if len(self.robot_ids) != 2 or len(base_frames) != 2:
            raise ValueError('approach goal generation currently requires exactly two robots')
        self.base_frames = dict(zip(self.robot_ids, base_frames))
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.target = None
        self.maps = {}
        self.published_goals = {}

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pubs = {
            robot_id: self.create_publisher(
                NavigationTask, f'/search/approach_task/{robot_id}', latched
            )
            for robot_id in self.robot_ids
        }
        self.map_subs = [
            self.create_subscription(
                OccupancyGrid,
                f'/{robot_id}/esdf_map',
                lambda msg, rid=robot_id: self._on_map(rid, msg),
                latched,
            )
            for robot_id in self.robot_ids
        ]
        self.target_sub = self.create_subscription(
            TargetObservation,
            str(self.get_parameter('target_topic').value),
            self._on_target,
            latched,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_map(self, robot_id, message):
        expected = int(message.info.width) * int(message.info.height)
        if expected and len(message.data) == expected:
            self.maps[robot_id] = message
            self._try_generate()

    def _on_target(self, message):
        if message.header.frame_id != self.global_frame:
            self.get_logger().error('Rejected target outside the global frame')
            return
        if self.target is None or message.mission_id != self.target.mission_id:
            self.published_goals.clear()
        self.target = message
        self._try_generate()

    def _try_generate(self):
        if self.target is None or len(self.maps) != len(self.robot_ids):
            return
        robot_positions = {}
        try:
            for robot_id, base_frame in self.base_frames.items():
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    base_frame,
                    Time(),
                    timeout=Duration(seconds=float(self.get_parameter('tf_timeout_sec').value)),
                )
                robot_positions[robot_id] = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                )
        except TransformException as exc:
            self.get_logger().warning(f'Waiting for robot TF: {exc}')
            return
        costmaps = {}
        for robot_id, message in self.maps.items():
            if abs(message.info.origin.orientation.z) > 1e-6:
                self.get_logger().error('Rotated ESDF origins are unsupported')
                return
            costmaps[robot_id] = {
                'costs': np.asarray(message.data, dtype=np.int16).reshape(
                    int(message.info.height), int(message.info.width)
                ),
                'origin_x': message.info.origin.position.x,
                'origin_y': message.info.origin.position.y,
                'resolution': message.info.resolution,
            }
        planner_kwargs = {
            'target_xy': (self.target.position.x, self.target.position.y),
            'radii': self.get_parameter('radii').value,
            'angle_samples': int(self.get_parameter('angle_samples').value),
            'max_goal_cost': int(self.get_parameter('max_goal_cost').value),
            'lethal_cost': int(self.get_parameter('lethal_cost').value),
        }
        goals = {}
        if not self.published_goals:
            try:
                goals = generate_approach_pair(
                    robot_positions=robot_positions,
                    costmaps=costmaps,
                    min_pair_separation=float(
                        self.get_parameter('min_pair_separation').value
                    ),
                    **planner_kwargs,
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f'No complete approach pair yet ({exc}); trying sequential fallback'
                )
        for robot_id in self.robot_ids:
            if robot_id in self.published_goals or robot_id in goals:
                continue
            try:
                goals[robot_id] = generate_approach_goal(
                    robot_position=robot_positions[robot_id],
                    costmap=costmaps[robot_id],
                    reserved_poses=[
                        *self.published_goals.values(), *goals.values()
                    ],
                    min_separation=float(
                        self.get_parameter('min_pair_separation').value
                    ),
                    **planner_kwargs,
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f'No sequential approach goal for {robot_id}: {exc}'
                )
        if not goals:
            return
        stamp = self.get_clock().now().to_msg()
        for robot_id, (x, y, yaw) in goals.items():
            message = NavigationTask()
            message.header.stamp = stamp
            message.header.frame_id = self.global_frame
            message.mission_id = self.target.mission_id
            message.goal_id = f'{self.target.mission_id}:{robot_id}:approach'
            message.robot_id = robot_id
            message.goal.position.x = x
            message.goal.position.y = y
            message.goal.orientation.z = math.sin(yaw / 2.0)
            message.goal.orientation.w = math.cos(yaw / 2.0)
            message.timeout_sec = float(
                self.get_parameter('navigation_timeout_sec').value
            )
            self.goal_pubs[robot_id].publish(message)
            self.published_goals[robot_id] = (x, y, yaw)
        self.get_logger().info(f'Published approach goals: {goals}')


def main(args=None):
    rclpy.init(args=args)
    node = ApproachGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
