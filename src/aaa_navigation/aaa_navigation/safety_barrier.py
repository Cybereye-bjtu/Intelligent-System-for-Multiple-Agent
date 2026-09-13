import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .algorithms import transform_polar_points, yaw_from_quaternion


class SafetyBarrier(Node):
    """Last-command safety filter with braking-distance and watchdog barriers."""

    def __init__(self):
        super().__init__("safety_barrier")
        defaults = {
            "scan_topic": "/scan_filtered",
            "odom_topic": "/odom",
            "nominal_cmd_topic": "/cmd_vel_nominal",
            "output_cmd_topic": "/cmd_vel",
            "control_frequency": 20.0,
            "scan_timeout": 0.35,
            "command_timeout": 0.35,
            "odom_timeout": 0.5,
            "base_clearance": 0.16,
            "reaction_time": 0.40,
            "braking_deceleration": 0.30,
            "hard_stop_distance": 0.20,
            "slow_margin": 0.25,
            "front_half_angle_deg": 45.0,
            "side_angle_min_deg": 30.0,
            "side_angle_max_deg": 100.0,
            "escape_angular_velocity": 0.30,
            "allow_escape_rotation": True,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.30,
            "robot_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.latest_scan = None
        self.latest_command = Twist()
        self.measured_speed = 0.0
        self.scan_time = None
        self.command_time = None
        self.odom_time = None
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_topic").value), 1
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/safety/debug/markers", 1
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.command_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("nominal_cmd_topic").value),
            self.command_callback,
            1,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        frequency = max(
            1.0, float(self.get_parameter("control_frequency").value)
        )
        self.timer = self.create_timer(1.0 / frequency, self.filter_command)

    def scan_callback(self, message):
        self.latest_scan = message
        self.scan_time = self.get_clock().now()

    def odom_callback(self, message):
        self.measured_speed = float(message.twist.twist.linear.x)
        self.odom_time = self.get_clock().now()

    def command_callback(self, message):
        self.latest_command = message
        self.command_time = self.get_clock().now()

    def _stale(self, stamp, parameter):
        if stamp is None:
            return True
        return (
            self.get_clock().now() - stamp
        ).nanoseconds / 1e9 > float(self.get_parameter(parameter).value)

    def _publish_zero(self):
        self.publisher.publish(Twist())

    def _scan_in_robot_frame(self):
        ranges = np.asarray(self.latest_scan.ranges, dtype=np.float64)
        angles = (
            self.latest_scan.angle_min
            + np.arange(ranges.size) * self.latest_scan.angle_increment
        )
        valid = np.isfinite(ranges) & (
            ranges >= self.latest_scan.range_min
        )
        scan_frame = (
            self.latest_scan.header.frame_id
            or str(self.get_parameter("robot_frame").value)
        )
        robot_frame = str(self.get_parameter("robot_frame").value)
        if scan_frame != robot_frame:
            transform = self.tf_buffer.lookup_transform(
                robot_frame,
                scan_frame,
                Time(),
                timeout=Duration(seconds=0.08),
            )
            safe_ranges = np.where(valid, ranges, 0.0)
            ranges, angles = transform_polar_points(
                safe_ranges,
                angles,
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation),
            )
        return ranges, angles, valid

    def _publish_safety_marker(
        self, safe_distance, front_distance, state, left_clearance, right_clearance
    ):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        half_angle = math.radians(
            float(self.get_parameter("front_half_angle_deg").value)
        )
        wedge = Marker()
        wedge.header.frame_id = str(self.get_parameter("robot_frame").value)
        wedge.header.stamp = stamp
        wedge.ns = "aaa_safety_zone"
        wedge.id = 0
        wedge.type = Marker.LINE_STRIP
        wedge.action = Marker.ADD
        wedge.pose.orientation.w = 1.0
        wedge.scale.x = 0.045
        wedge.points.append(Point(x=0.0, y=0.0, z=0.12))
        for angle in np.linspace(-half_angle, half_angle, 25):
            wedge.points.append(
                Point(
                    x=safe_distance * math.cos(angle),
                    y=safe_distance * math.sin(angle),
                    z=0.12,
                )
            )
        wedge.points.append(Point(x=0.0, y=0.0, z=0.12))
        if state == "STOP":
            wedge.color.r, wedge.color.g = 1.0, 0.0
        elif state == "SLOW":
            wedge.color.r, wedge.color.g = 1.0, 0.75
        else:
            wedge.color.r, wedge.color.g = 0.1, 1.0
        wedge.color.a = 0.95
        markers.markers.append(wedge)

        text = Marker()
        text.header.frame_id = wedge.header.frame_id
        text.header.stamp = stamp
        text.ns = "aaa_safety_status"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.25
        text.pose.position.y = 0.0
        text.pose.position.z = 0.75
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color = wedge.color
        text.text = (
            f"Safety {state}\nfront {front_distance:.2f} m / "
            f"limit {safe_distance:.2f} m\n"
            f"left {left_clearance:.2f} m  right {right_clearance:.2f} m"
        )
        markers.markers.append(text)
        self.marker_publisher.publish(markers)

    def filter_command(self):
        if (
            self._stale(self.scan_time, "scan_timeout")
            or self._stale(self.command_time, "command_timeout")
            or self._stale(self.odom_time, "odom_timeout")
            or self.latest_scan is None
        ):
            self._publish_zero()
            self._publish_safety_marker(0.5, 0.0, "STOP", 0.0, 0.0)
            return

        try:
            ranges, angles, valid = self._scan_in_robot_frame()
        except TransformException as error:
            self.get_logger().warning(
                f"Safety barrier waiting for scan TF: {error}",
                throttle_duration_sec=2.0,
            )
            self._publish_zero()
            return
        front_angle = math.radians(
            float(self.get_parameter("front_half_angle_deg").value)
        )
        front = ranges[valid & (np.abs(angles) <= front_angle)]
        front_distance = float(np.min(front)) if front.size else 0.0

        side_min = math.radians(
            float(self.get_parameter("side_angle_min_deg").value)
        )
        side_max = math.radians(
            float(self.get_parameter("side_angle_max_deg").value)
        )
        left = ranges[
            valid & (angles >= side_min) & (angles <= side_max)
        ]
        right = ranges[
            valid & (angles <= -side_min) & (angles >= -side_max)
        ]
        left_clearance = float(np.min(left)) if left.size else 0.0
        right_clearance = float(np.min(right)) if right.size else 0.0

        command = Twist()
        command.linear.x = float(
            np.clip(
                self.latest_command.linear.x,
                0.0,
                float(self.get_parameter("max_linear_velocity").value),
            )
        )
        command.angular.z = float(
            np.clip(
                self.latest_command.angular.z,
                -float(self.get_parameter("max_angular_velocity").value),
                float(self.get_parameter("max_angular_velocity").value),
            )
        )
        speed = max(0.0, self.measured_speed, command.linear.x)
        deceleration = max(
            0.05, float(self.get_parameter("braking_deceleration").value)
        )
        safe_distance = (
            float(self.get_parameter("base_clearance").value)
            + speed * float(self.get_parameter("reaction_time").value)
            + speed * speed / (2.0 * deceleration)
        )
        hard_stop = float(self.get_parameter("hard_stop_distance").value)
        slow_margin = max(
            0.05, float(self.get_parameter("slow_margin").value)
        )

        state = "CLEAR"
        if front_distance <= hard_stop:
            state = "STOP"
            command.linear.x = 0.0
            if bool(self.get_parameter("allow_escape_rotation").value):
                escape = float(
                    self.get_parameter("escape_angular_velocity").value
                )
                if abs(command.angular.z) < escape:
                    command.angular.z = (
                        escape
                        if left_clearance >= right_clearance
                        else -escape
                    )
            else:
                command.angular.z = 0.0
        elif front_distance < safe_distance + slow_margin:
            state = "SLOW"
            scale = np.clip(
                (front_distance - safe_distance) / slow_margin, 0.0, 1.0
            )
            command.linear.x *= float(scale)

        self.publisher.publish(command)
        self._publish_safety_marker(
            safe_distance,
            front_distance,
            state,
            left_clearance,
            right_clearance,
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyBarrier()
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
