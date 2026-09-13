import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import UInt64


class CloudCommandSender(Node):
    """Stamp cloud controller output and publish an independent heartbeat."""

    def __init__(self):
        super().__init__("cloud_command_sender")
        defaults = {
            "input_topic": "/cmd_vel_cloud_raw",
            "output_topic": "/cmd_vel_cloud",
            "heartbeat_topic": "/cloud_heartbeat",
            "publish_frequency": 10.0,
            "heartbeat_frequency": 5.0,
            "input_timeout": 0.30,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.30,
            "base_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.latest_command = Twist()
        self.command_time = None
        self.heartbeat_sequence = 0
        self.command_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("output_topic").value),
            1,
        )
        self.heartbeat_publisher = self.create_publisher(
            UInt64,
            str(self.get_parameter("heartbeat_topic").value),
            1,
        )
        self.subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self.command_callback,
            1,
        )
        publish_frequency = max(
            1.0, float(self.get_parameter("publish_frequency").value)
        )
        heartbeat_frequency = max(
            1.0,
            float(self.get_parameter("heartbeat_frequency").value),
        )
        self.command_timer = self.create_timer(
            1.0 / publish_frequency, self.publish_command
        )
        self.heartbeat_timer = self.create_timer(
            1.0 / heartbeat_frequency, self.publish_heartbeat
        )

    def command_callback(self, message):
        self.latest_command = message
        self.command_time = self.get_clock().now()

    def _input_is_fresh(self):
        if self.command_time is None:
            return False
        age = (
            self.get_clock().now() - self.command_time
        ).nanoseconds / 1e9
        return age <= float(self.get_parameter("input_timeout").value)

    def publish_command(self):
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = str(
            self.get_parameter("base_frame").value
        )
        command = self.latest_command
        finite = math.isfinite(command.linear.x) and math.isfinite(
            command.angular.z
        )
        if self._input_is_fresh() and finite:
            linear_limit = float(
                self.get_parameter("max_linear_velocity").value
            )
            angular_limit = float(
                self.get_parameter("max_angular_velocity").value
            )
            output.twist.linear.x = max(
                0.0, min(float(command.linear.x), linear_limit)
            )
            output.twist.angular.z = max(
                -angular_limit,
                min(float(command.angular.z), angular_limit),
            )
        self.command_publisher.publish(output)

    def publish_heartbeat(self):
        self.heartbeat_sequence = (
            self.heartbeat_sequence + 1
        ) % (2**64)
        message = UInt64()
        message.data = self.heartbeat_sequence
        self.heartbeat_publisher.publish(message)

    def publish_zero(self):
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = str(
            self.get_parameter("base_frame").value
        )
        self.command_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CloudCommandSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
