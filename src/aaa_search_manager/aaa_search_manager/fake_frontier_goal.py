from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class FakeFrontierGoalPublisher(Node):
    def __init__(self, robot: str, x: float, y: float, frame_id: str) -> None:
        super().__init__('fake_frontier_goal')
        self.topic = f'/search/exploration_goal/{robot}'
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)
        self.msg = PoseStamped()
        self.msg.header.frame_id = frame_id
        self.msg.pose.position.x = x
        self.msg.pose.position.y = y
        self.msg.pose.orientation.w = 1.0
        self.timer = self.create_timer(0.3, self._publish_once)
        self.done = False

    def _publish_once(self) -> None:
        if self.done:
            return
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info(
            f'Published fake frontier goal to {self.topic} at '
            f'({self.msg.pose.position.x:.3f}, {self.msg.pose.position.y:.3f})'
        )
        self.done = True
        self.timer.cancel()


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', default='hyzx001')
    parser.add_argument('--x', type=float, default=1.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--frame-id', default='site_map')
    known, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = FakeFrontierGoalPublisher(known.robot, known.x, known.y, known.frame_id)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
