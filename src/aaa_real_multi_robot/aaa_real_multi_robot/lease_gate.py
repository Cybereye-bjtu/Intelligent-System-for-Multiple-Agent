import math

import rclpy
from aaa_real_multi_robot_interfaces.msg import NavigationLease
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class LeaseGate(Node):
    """Vehicle-side final fleet permission gate; startup and all stale states stop."""

    def __init__(self):
        super().__init__("lease_gate")
        self.declare_parameter("input_topic", "cmd_vel_safe")
        self.declare_parameter("output_topic", "cmd_vel_leased")
        self.declare_parameter("lease_topic", "fleet/lease")
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("lease_receipt_timeout", 0.50)
        self.declare_parameter("publish_frequency", 20.0)
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.20)
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("global_frame", "site_map")
        self.publisher = self.create_publisher(Twist, str(self.get_parameter("output_topic").value), 5)
        self.create_subscription(Twist, str(self.get_parameter("input_topic").value), self._command_callback, 5)
        self.create_subscription(NavigationLease, str(self.get_parameter("lease_topic").value), self._lease_callback, 5)
        stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/swarm/emergency_stop", self._stop_callback, stop_qos)
        self.create_service(SetBool, "~/enable", self._enable_callback)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.command = Twist()
        self.command_receipt = None
        self.lease = None
        self.lease_receipt = None
        self.emergency_stop = True
        self.enabled = False
        self.map_epoch = 0
        frequency = max(5.0, float(self.get_parameter("publish_frequency").value))
        self.create_timer(1.0 / frequency, self._tick, clock=self.steady_clock)

    def _command_callback(self, message):
        self.command = message
        self.command_receipt = self.steady_clock.now()

    def _lease_callback(self, message):
        if int(message.robot_id) != int(self.get_parameter("robot_id").value):
            return
        if message.header.frame_id != str(self.get_parameter("global_frame").value):
            return
        limits = (float(message.max_linear_speed), float(message.max_angular_speed))
        if not all(math.isfinite(value) and value >= 0.0 for value in limits):
            return
        if int(message.map_epoch) < self.map_epoch:
            return
        self.map_epoch = int(message.map_epoch)
        self.lease = message
        self.lease_receipt = self.steady_clock.now()

    def _stop_callback(self, message):
        self.emergency_stop = bool(message.data)

    def _enable_callback(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = "lease gate armed" if self.enabled else "lease gate locked"
        if not self.enabled:
            self.publisher.publish(Twist())
        return response

    def _fresh(self, receipt, timeout):
        return receipt is not None and (self.steady_clock.now() - receipt) <= Duration(seconds=timeout)

    def _lease_valid(self):
        if self.lease is None or not self.lease.permit_motion:
            return False
        expiry = Time.from_msg(self.lease.valid_until)
        return self.get_clock().now() < expiry

    @staticmethod
    def _bounded(value, limit):
        value = float(value)
        limit = max(0.0, float(limit))
        return max(-limit, min(limit, value)) if math.isfinite(value) else 0.0

    def _tick(self):
        permit = (
            self.enabled
            and not self.emergency_stop
            and self._fresh(self.command_receipt, float(self.get_parameter("command_timeout").value))
            and self._fresh(self.lease_receipt, float(self.get_parameter("lease_receipt_timeout").value))
            and self._lease_valid()
        )
        output = Twist()
        if permit:
            linear_limit = min(float(self.get_parameter("max_linear_speed").value), float(self.lease.max_linear_speed))
            angular_limit = min(float(self.get_parameter("max_angular_speed").value), float(self.lease.max_angular_speed))
            requested_x = self._bounded(self.command.linear.x, linear_limit)
            requested_y = self._bounded(self.command.linear.y, linear_limit)
            magnitude = math.hypot(requested_x, requested_y)
            scale = 1.0 if magnitude <= linear_limit or magnitude == 0.0 else linear_limit / magnitude
            output.linear.x = requested_x * scale
            output.linear.y = requested_y * scale
            output.angular.z = self._bounded(self.command.angular.z, angular_limit)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LeaseGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
