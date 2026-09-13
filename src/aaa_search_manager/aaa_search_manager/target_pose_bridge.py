from __future__ import annotations

from functools import partial
from typing import Dict

import rclpy
from aaa_search_interfaces.msg import TargetObservation
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .target_transform import transform_point_and_covariance, validate_observation_values


class TargetPoseBridge(Node):
    """Transform confirmed per-robot target observations into ``site_map``."""

    def __init__(self) -> None:
        super().__init__('target_pose_bridge')
        self.declare_parameter('robot_ids', ['hyzx001', 'jetson003'])
        self.declare_parameter(
            'base_frames', ['hyzx001/base_footprint', 'jetson003/base_link']
        )
        self.declare_parameter('global_frame', 'site_map')
        self.declare_parameter('input_topic_prefix', '/search/target_observation')
        self.declare_parameter('target_found_topic', '/search/target_found')
        self.declare_parameter(
            'global_observation_topic', '/search/target_observation_global'
        )
        self.declare_parameter('min_semantic_confidence', 0.80)
        self.declare_parameter('min_valid_depth_ratio', 0.35)
        self.declare_parameter('min_observation_count', 2)
        self.declare_parameter('tf_timeout_sec', 0.50)

        robot_ids = [str(value) for value in self.get_parameter('robot_ids').value]
        base_frames = [str(value) for value in self.get_parameter('base_frames').value]
        if len(robot_ids) != len(base_frames) or not robot_ids:
            raise ValueError('robot_ids and base_frames must be non-empty parallel lists')
        self.expected_frames: Dict[str, str] = dict(zip(robot_ids, base_frames))
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.min_confidence = float(
            self.get_parameter('min_semantic_confidence').value
        )
        self.min_depth_ratio = float(
            self.get_parameter('min_valid_depth_ratio').value
        )
        self.min_observations = int(self.get_parameter('min_observation_count').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.target_found_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('target_found_topic').value),
            latched_qos,
        )
        self.global_observation_pub = self.create_publisher(
            TargetObservation,
            str(self.get_parameter('global_observation_topic').value),
            latched_qos,
        )
        # Target inference is asynchronous; keep the transform that corresponds
        # to the original RGB-D acquisition time until the result arrives.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=300.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        prefix = str(self.get_parameter('input_topic_prefix').value).rstrip('/')
        self._observation_subscriptions = []
        for robot_id in robot_ids:
            self._observation_subscriptions.append(
                self.create_subscription(
                    TargetObservation,
                    f'{prefix}/{robot_id}',
                    partial(self._on_observation, robot_id),
                    10,
                )
            )
        self.get_logger().info(
            f'Target pose bridge ready: {self.expected_frames} -> {self.global_frame}'
        )

    def _on_observation(self, expected_robot_id: str, msg: TargetObservation) -> None:
        expected_frame = self.expected_frames[expected_robot_id]
        if not msg.mission_id.strip():
            self.get_logger().error('Rejected target observation with empty mission_id')
            return
        if msg.robot_id != expected_robot_id:
            self.get_logger().error(
                f'Rejected robot_id={msg.robot_id!r} on {expected_robot_id!r} topic'
            )
            return
        if msg.header.frame_id != expected_frame:
            self.get_logger().error(
                f'Rejected frame_id={msg.header.frame_id!r}; expected {expected_frame!r}'
            )
            return
        try:
            point, covariance = validate_observation_values(
                position=(msg.position.x, msg.position.y, msg.position.z),
                covariance=msg.position_covariance,
                semantic_confidence=msg.semantic_confidence,
                valid_depth_ratio=msg.valid_depth_ratio,
                observation_count=msg.observation_count,
                min_semantic_confidence=self.min_confidence,
                min_valid_depth_ratio=self.min_depth_ratio,
                min_observation_count=self.min_observations,
            )
            capture_time = Time.from_msg(msg.header.stamp)
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                expected_frame,
                capture_time,
                timeout=Duration(seconds=self.tf_timeout),
            )
            tr = transform.transform.translation
            qr = transform.transform.rotation
            global_point, global_covariance = transform_point_and_covariance(
                point,
                covariance,
                (tr.x, tr.y, tr.z),
                (qr.x, qr.y, qr.z, qr.w),
            )
        except (ValueError, TransformException) as exc:
            self.get_logger().error(f'Rejected target observation: {exc}')
            return

        output = TargetObservation()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.global_frame
        output.mission_id = msg.mission_id
        output.robot_id = msg.robot_id
        output.target_label = msg.target_label
        output.instance_id = msg.instance_id
        output.position.x, output.position.y, output.position.z = (
            float(value) for value in global_point
        )
        output.position_covariance = global_covariance.reshape(-1).tolist()
        output.semantic_confidence = msg.semantic_confidence
        output.valid_depth_ratio = msg.valid_depth_ratio
        output.observation_count = msg.observation_count
        self.global_observation_pub.publish(output)

        pose = PoseStamped()
        pose.header = output.header
        pose.pose.position = output.position
        pose.pose.orientation.w = 1.0
        self.target_found_pub.publish(pose)
        self.get_logger().info(
            f'Published target {msg.target_label!r} from {msg.robot_id} at '
            f'({global_point[0]:.3f}, {global_point[1]:.3f}, {global_point[2]:.3f}) '
            f'in {self.global_frame}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
