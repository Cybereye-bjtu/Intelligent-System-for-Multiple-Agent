import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import UInt64
from std_srvs.srv import SetBool

from .cloud_protocol import stamp_to_nanoseconds, validate_command_stamp


class EdgeCommandReceiver(Node):
    """Fail-closed receiver for timestamped commands from the cloud."""

    def __init__(self):
        super().__init__("edge_command_receiver")
        defaults = {
            "command_topic": "/cmd_vel_cloud",
            "heartbeat_topic": "/cloud_heartbeat",
            "output_topic": "/cmd_vel_edge_nominal",
            "status_topic": "/aaa/network_status",
            "control_frequency": 20.0,
            "status_frequency": 2.0,
            "command_timeout": 0.35,
            "heartbeat_timeout": 0.60,
            "max_source_age": 0.30,
            "max_future_skew": 0.10,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.45,
            "armed_on_start": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.armed = bool(self.get_parameter("armed_on_start").value)
        self.fault_latched = not self.armed
        self.fault_reason = "edge receiver requires manual arm"
        self.latest_command = Twist()
        self.command_receipt_time = None
        self.heartbeat_receipt_time = None
        self.last_command_stamp = None
        self.last_heartbeat_sequence = None

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 1
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            1,
        )
        self.command_subscription = self.create_subscription(
            TwistStamped,
            str(self.get_parameter("command_topic").value),
            self.command_callback,
            1,
        )
        self.heartbeat_subscription = self.create_subscription(
            UInt64,
            str(self.get_parameter("heartbeat_topic").value),
            self.heartbeat_callback,
            1,
        )
        self.arm_service = self.create_service(
            SetBool, "~/arm", self.arm_callback
        )
        control_frequency = max(
            1.0, float(self.get_parameter("control_frequency").value)
        )
        status_frequency = max(
            0.2, float(self.get_parameter("status_frequency").value)
        )
        self.control_timer = self.create_timer(
            1.0 / control_frequency, self.control
        )
        self.status_timer = self.create_timer(
            1.0 / status_frequency, self.publish_status
        )
        self.get_logger().warning(
            "Cloud command receiver is DISARMED. A network fault remains "
            "latched until /edge_command_receiver/arm is called."
        )

    def _age(self, receipt_time):
        if receipt_time is None:
            return float("inf")
        return (
            self.get_clock().now() - receipt_time
        ).nanoseconds / 1e9

    def _heartbeat_is_fresh(self):
        return self._age(self.heartbeat_receipt_time) <= float(
            self.get_parameter("heartbeat_timeout").value
        )

    def _command_is_fresh(self):
        return self._age(self.command_receipt_time) <= float(
            self.get_parameter("command_timeout").value
        )

    def _trip(self, reason):
        was_armed = self.armed
        self.armed = False
        self.fault_latched = True
        self.fault_reason = reason
        self.latest_command = Twist()
        self.publisher.publish(Twist())
        if was_armed:
            self.get_logger().error(
                f"Cloud command fault latched: {reason}"
            )

    def command_callback(self, message):
        now = self.get_clock().now()
        stamp = stamp_to_nanoseconds(message.header.stamp)
        valid_stamp, reason = validate_command_stamp(
            now.nanoseconds,
            stamp,
            float(self.get_parameter("max_source_age").value),
            float(self.get_parameter("max_future_skew").value),
            self.last_command_stamp,
        )
        command = message.twist
        finite = math.isfinite(command.linear.x) and math.isfinite(
            command.angular.z
        )
        if not valid_stamp or not finite:
            if self.armed:
                self._trip(reason if not valid_stamp else "non-finite command")
            return

        self.last_command_stamp = stamp
        self.command_receipt_time = now
        linear_limit = float(
            self.get_parameter("max_linear_velocity").value
        )
        angular_limit = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.latest_command = Twist()
        self.latest_command.linear.x = max(
            0.0, min(float(command.linear.x), linear_limit)
        )
        self.latest_command.angular.z = max(
            -angular_limit,
            min(float(command.angular.z), angular_limit),
        )

    def heartbeat_callback(self, message):
        sequence = int(message.data)
        if (
            self.last_heartbeat_sequence is not None
            and sequence <= self.last_heartbeat_sequence
            and self.armed
        ):
            self._trip("cloud heartbeat sequence restarted or regressed")
        self.last_heartbeat_sequence = sequence
        self.heartbeat_receipt_time = self.get_clock().now()

    def arm_callback(self, request, response):
        if not request.data:
            self._trip("manually disarmed")
            response.success = True
            response.message = "edge command receiver disarmed and zeroed"
            return response

        if not self._heartbeat_is_fresh():
            response.success = False
            response.message = "cannot arm: cloud heartbeat is stale"
            return response
        if not self._command_is_fresh():
            response.success = False
            response.message = "cannot arm: cloud command is stale"
            return response

        self.armed = True
        self.fault_latched = False
        self.fault_reason = ""
        response.success = True
        response.message = "edge command receiver armed"
        self.get_logger().warning(response.message)
        return response

    def control(self):
        if self.armed and not self._heartbeat_is_fresh():
            self._trip("cloud heartbeat timeout")
        elif self.armed and not self._command_is_fresh():
            self._trip("cloud command timeout")
        self.publisher.publish(
            self.latest_command if self.armed else Twist()
        )

    def publish_status(self):
        command_age = self._age(self.command_receipt_time)
        heartbeat_age = self._age(self.heartbeat_receipt_time)
        status = DiagnosticStatus()
        status.name = "aaa_cloud_link"
        status.hardware_id = "hyzx_edge"
        status.level = (
            DiagnosticStatus.OK if self.armed else DiagnosticStatus.ERROR
        )
        status.message = (
            "ARMED"
            if self.armed
            else f"STOPPED: {self.fault_reason}"
        )
        status.values = [
            KeyValue(key="armed", value=str(self.armed)),
            KeyValue(
                key="fault_latched", value=str(self.fault_latched)
            ),
            KeyValue(
                key="command_age_sec",
                value=(
                    f"{command_age:.3f}"
                    if math.isfinite(command_age)
                    else "inf"
                ),
            ),
            KeyValue(
                key="heartbeat_age_sec",
                value=(
                    f"{heartbeat_age:.3f}"
                    if math.isfinite(heartbeat_age)
                    else "inf"
                ),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = EdgeCommandReceiver()
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
