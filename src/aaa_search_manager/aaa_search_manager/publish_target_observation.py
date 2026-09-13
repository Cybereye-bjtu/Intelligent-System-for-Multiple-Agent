from __future__ import annotations

import argparse
import time

import rclpy
from aaa_search_interfaces.msg import TargetObservation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .target_observation_contract import load_target_observation


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='Publish a validated CyberEye target_pose_result.json to ROS 2.'
    )
    parser.add_argument('result_json')
    parser.add_argument('--settle-seconds', type=float, default=0.75)
    return parser.parse_args(args)


def main(args=None) -> None:
    cli = parse_args(args)
    value = load_target_observation(cli.result_json)
    rclpy.init()
    node = Node('target_observation_publisher')
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    topic = f"/search/target_observation/{value['robot_id']}"
    publisher = node.create_publisher(TargetObservation, topic, qos)
    message = TargetObservation()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = value['frame_id']
    message.mission_id = value['mission_id']
    message.robot_id = value['robot_id']
    message.target_label = str(value.get('target_label') or '')
    message.instance_id = str(value.get('instance_id') or '')
    message.position.x, message.position.y, message.position.z = (
        float(component) for component in value['position']
    )
    message.position_covariance = value['position_covariance']
    message.semantic_confidence = float(value['semantic_confidence'])
    message.valid_depth_ratio = float(value['valid_depth_ratio'])
    message.observation_count = int(value['observation_count'])

    # Give discovery a bounded interval, publish more than once, then exit.
    deadline = time.monotonic() + max(0.1, cli.settle_seconds)
    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(
        f"Published mission={message.mission_id!r} robot={message.robot_id!r} to {topic}"
    )
    node.destroy_node()
    rclpy.shutdown()
