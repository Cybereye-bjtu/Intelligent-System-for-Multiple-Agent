import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVelGateway(Node):
    """Fail-closed speed limiter between AAA navigation and the real chassis."""

    def __init__(self):
        super().__init__("aaa_cmd_gateway")
        defaults = {
            "input_topic": "/cmd_vel_aaa",
            "output_topic": "/cmd_vel",
            "control_frequency": 20.0,
            "command_timeout": 0.35,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.45,
            "enabled_on_start": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.enabled = bool(self.get_parameter("enabled_on_start").value)
        self.latest_command = Twist()
        self.command_time = None
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 1
        )
        self.subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self.command_callback,
            1,
        )
        self.enable_service = self.create_service(
            SetBool, "~/enable", self.enable_callback
        )
        frequency = max(
            1.0, float(self.get_parameter("control_frequency").value)
        )
        self.timer = self.create_timer(1.0 / frequency, self.publish_command)
        self.get_logger().warning(
            "Chassis output is DISABLED. Call /aaa_cmd_gateway/enable "
            "after completing the pre-drive checks."
        )

    def command_callback(self, message: Twist):
        self.latest_command = message
        self.command_time = self.get_clock().now()

    def enable_callback(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.latest_command = Twist()
            self.command_time = None
            self.publisher.publish(Twist())
        response.success = True
        response.message = (
            "AAA chassis output enabled"
            if self.enabled
            else "AAA chassis output disabled and zeroed"
        )
        self.get_logger().warning(response.message)
        return response

    def _command_is_fresh(self):
        if self.command_time is None:
            return False
        age = (
            self.get_clock().now() - self.command_time
        ).nanoseconds / 1e9
        return age <= float(self.get_parameter("command_timeout").value)

    def publish_command(self):
        output = Twist()
        command = self.latest_command
        finite = math.isfinite(command.linear.x) and math.isfinite(
            command.angular.z
        )
        if self.enabled and self._command_is_fresh() and finite:
            linear_limit = float(
                self.get_parameter("max_linear_velocity").value
            )
            angular_limit = float(
                self.get_parameter("max_angular_velocity").value
            )
            output.linear.x = max(
                0.0, min(float(command.linear.x), linear_limit)
            )
            output.angular.z = max(
                -angular_limit,
                min(float(command.angular.z), angular_limit),
            )
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
