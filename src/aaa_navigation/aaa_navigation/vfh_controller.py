import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .algorithms import (
    choose_vfh_heading,
    choose_vfh_recovery_heading,
    corner_aware_target,
    fuzzy_velocity,
    regulated_angular_velocity,
    transform_polar_points,
    update_alignment_state,
    wrap_angle,
    yaw_from_quaternion,
)


class VfhController(Node):
    def __init__(self):
        super().__init__("vfh_controller")
        defaults = {
            "scan_topic": "/scan_filtered",
            "path_topic": "/global_path",
            "nominal_cmd_topic": "/cmd_vel_nominal",
            "map_frame": "map",
            "robot_frame": "base_link",
            "control_frequency": 10.0,
            "scan_timeout": 0.75,
            "lookahead_distance": 0.85,
            "corner_search_distance": 1.5,
            "corner_angle_threshold_deg": 25.0,
            "outward_shift_gain": 0.22,
            "outward_shift_max": 0.25,
            "goal_tolerance": 0.25,
            "sectors": 120,
            "histogram_max_range": 4.0,
            "obstacle_threshold": 0.52,
            "robot_width": 0.212,
            "valley_safety_margin_deg": 8.0,
            "heading_weight": 5.0,
            "previous_heading_weight": 2.0,
            "clearance_weight": 1.5,
            "max_linear_velocity": 0.10,
            "min_effective_linear_velocity": 0.0,
            "max_angular_velocity": 0.30,
            "max_linear_acceleration": 0.15,
            "max_angular_acceleration": 0.30,
            "max_angular_deceleration": 0.80,
            "angular_proportional_gain": 0.80,
            "align_enter_angle_deg": 25.0,
            "align_exit_angle_deg": 8.0,
            "heading_deadband_deg": 3.0,
            "pose_timeout": 0.20,
            # Visual-only scale for RViz debug markers. The original marker
            # dimensions were intended for a much larger robot and obscured
            # the path around the 0.277 x 0.212 m Jetson003 chassis.
            "debug_marker_scale": 0.40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.sectors = int(self.get_parameter("sectors").value)
        self.sector_width = 2.0 * math.pi / self.sectors
        self.max_range = float(
            self.get_parameter("histogram_max_range").value
        )
        self.max_linear = float(
            self.get_parameter("max_linear_velocity").value
        )
        self.min_effective_linear = min(
            self.max_linear,
            max(
                0.0,
                float(
                    self.get_parameter(
                        "min_effective_linear_velocity"
                    ).value
                ),
            ),
        )
        self.max_angular = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.max_linear_accel = float(
            self.get_parameter("max_linear_acceleration").value
        )
        self.max_angular_accel = float(
            self.get_parameter("max_angular_acceleration").value
        )
        self.max_angular_decel = float(
            self.get_parameter("max_angular_deceleration").value
        )
        self.debug_marker_scale = max(
            0.05, float(self.get_parameter("debug_marker_scale").value)
        )
        self.path = []
        self.scan = None
        self.scan_receipt_time = None
        self.previous_heading_global = None
        self.aligning = False
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.last_control_time = None

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_publisher = self.create_publisher(
            Twist, str(self.get_parameter("nominal_cmd_topic").value), 1
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/vfh/debug/markers", 1
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.path_subscription = self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            path_qos,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        frequency = float(self.get_parameter("control_frequency").value)
        self.timer = self.create_timer(1.0 / max(frequency, 1.0), self.control)

    def scan_callback(self, message: LaserScan):
        self.scan = message
        self.scan_receipt_time = self.get_clock().now()

    def path_callback(self, message: Path):
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        if len(self.path) < 2:
            self.aligning = False
            self.previous_heading_global = None

    def _pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.robot_frame,
            Time(),
            timeout=Duration(seconds=0.08),
        )
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > float(self.get_parameter("pose_timeout").value):
            raise RuntimeError(f"pose transform is stale by {age:.3f} s")
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )

    def _histogram(self):
        ranges = np.asarray(self.scan.ranges, dtype=np.float64)
        angles = (
            self.scan.angle_min
            + np.arange(ranges.size) * self.scan.angle_increment
        )
        valid = np.isfinite(ranges) & (ranges >= self.scan.range_min)
        effective = np.where(
            valid, np.minimum(ranges, self.max_range), self.max_range
        )
        scan_frame = self.scan.header.frame_id or self.robot_frame
        if scan_frame != self.robot_frame:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                scan_frame,
                Time(),
                timeout=Duration(seconds=0.08),
            )
            effective, angles = transform_polar_points(
                effective,
                angles,
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw_from_quaternion(transform.transform.rotation),
            )
            effective = np.minimum(effective, self.max_range)
        danger = np.clip((self.max_range - effective) / self.max_range, 0.0, 1.0)
        indices = np.floor((angles + math.pi) / self.sector_width).astype(int)
        inside = (indices >= 0) & (indices < self.sectors)
        histogram = np.zeros(self.sectors, dtype=np.float64)
        observed = np.zeros(self.sectors, dtype=bool)
        np.maximum.at(histogram, indices[inside], danger[inside])
        observed[indices[inside]] = True
        # Smooth only values with actual observations; missing rear sectors
        # remain explicitly unavailable in choose_vfh_heading.
        kernel = np.array([0.2, 0.6, 0.2])
        histogram = np.convolve(histogram, kernel, mode="same")
        return histogram, observed, angles, effective

    def _rate_limit(self, linear, angular):
        now = self.get_clock().now()
        if self.last_control_time is None:
            dt = 0.1
        else:
            dt = min(0.2, max(0.001, (now - self.last_control_time).nanoseconds / 1e9))
        linear = np.clip(
            linear,
            self.previous_linear - self.max_linear_accel * dt,
            self.previous_linear + self.max_linear_accel * dt,
        )
        angular_rate = (
            self.max_angular_decel
            if abs(angular) < abs(self.previous_angular)
            or angular * self.previous_angular < 0.0
            else self.max_angular_accel
        )
        angular = np.clip(
            angular,
            self.previous_angular - angular_rate * dt,
            self.previous_angular + angular_rate * dt,
        )
        self.previous_linear = float(linear)
        self.previous_angular = float(angular)
        self.last_control_time = now
        return float(linear), float(angular)

    def _publish_stop(self):
        self.previous_linear = 0.0
        self.previous_angular = 0.0
        self.cmd_publisher.publish(Twist())

    def _publish_markers(
        self, target, selected_angle, histogram, observed, obstacle_threshold
    ):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        visual_scale = self.debug_marker_scale
        target_marker = Marker()
        target_marker.header.frame_id = self.map_frame
        target_marker.header.stamp = stamp
        target_marker.ns = "aaa_control_target"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = target[0]
        target_marker.pose.position.y = target[1]
        target_marker.pose.orientation.w = 1.0
        target_size = 0.18 * visual_scale
        target_marker.scale.x = target_size
        target_marker.scale.y = target_size
        target_marker.scale.z = target_size
        target_marker.color.r = 1.0
        target_marker.color.g = 0.55
        target_marker.color.a = 1.0
        markers.markers.append(target_marker)

        arrow = Marker()
        arrow.header.frame_id = self.robot_frame
        arrow.header.stamp = stamp
        arrow.ns = "aaa_vfh_heading"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.15),
            Point(
                x=visual_scale * math.cos(selected_angle),
                y=visual_scale * math.sin(selected_angle),
                z=0.15,
            ),
        ]
        arrow.scale.x = 0.06 * visual_scale
        arrow.scale.y = 0.12 * visual_scale
        arrow.scale.z = 0.16 * visual_scale
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        markers.markers.append(arrow)

        histogram_marker = Marker()
        histogram_marker.header.frame_id = self.robot_frame
        histogram_marker.header.stamp = stamp
        histogram_marker.ns = "aaa_vfh_histogram"
        histogram_marker.id = 2
        histogram_marker.type = Marker.LINE_LIST
        histogram_marker.action = Marker.ADD
        histogram_marker.scale.x = 0.025 * visual_scale
        histogram_marker.pose.orientation.w = 1.0
        for index, danger in enumerate(histogram):
            angle = -math.pi + (index + 0.5) * self.sector_width
            inner_radius = 0.48 * visual_scale
            outer_radius = inner_radius + visual_scale * (
                0.12 + 0.75 * float(danger)
            )
            start = Point(
                x=inner_radius * math.cos(angle),
                y=inner_radius * math.sin(angle),
                z=0.08,
            )
            end = Point(
                x=outer_radius * math.cos(angle),
                y=outer_radius * math.sin(angle),
                z=0.08,
            )
            if not observed[index]:
                color = ColorRGBA(r=0.35, g=0.35, b=0.35, a=0.55)
            elif danger >= obstacle_threshold:
                color = ColorRGBA(r=1.0, g=0.05, b=0.05, a=0.90)
            else:
                color = ColorRGBA(r=0.05, g=0.9, b=0.20, a=0.75)
            histogram_marker.points.extend([start, end])
            histogram_marker.colors.extend([color, color])
        markers.markers.append(histogram_marker)

        target_arrow = Marker()
        target_arrow.header.frame_id = self.robot_frame
        target_arrow.header.stamp = stamp
        target_arrow.ns = "aaa_path_target_heading"
        target_arrow.id = 3
        target_arrow.type = Marker.ARROW
        target_arrow.action = Marker.ADD
        target_arrow.points = [
            Point(x=0.0, y=0.0, z=0.11),
            Point(
                x=0.85 * visual_scale * math.cos(self._last_target_angle),
                y=0.85 * visual_scale * math.sin(self._last_target_angle),
                z=0.11,
            ),
        ]
        target_arrow.scale.x = 0.025 * visual_scale
        target_arrow.scale.y = 0.07 * visual_scale
        target_arrow.scale.z = 0.09 * visual_scale
        target_arrow.color.r = 1.0
        target_arrow.color.g = 0.85
        target_arrow.color.a = 1.0
        markers.markers.append(target_arrow)
        self.marker_publisher.publish(markers)

    def control(self):
        scan_stale = self.scan_receipt_time is None or (
            self.get_clock().now() - self.scan_receipt_time
        ).nanoseconds / 1e9 > float(
            self.get_parameter("scan_timeout").value
        )
        if self.scan is None or scan_stale or len(self.path) < 2:
            self._publish_stop()
            return
        try:
            robot_x, robot_y, robot_yaw = self._pose()
        except (TransformException, RuntimeError) as error:
            self.get_logger().warning(
                f"Controller waiting for TF: {error}",
                throttle_duration_sec=2.0,
            )
            self._publish_stop()
            return
        goal_distance = math.hypot(
            self.path[-1][0] - robot_x, self.path[-1][1] - robot_y
        )
        if goal_distance <= float(self.get_parameter("goal_tolerance").value):
            self._publish_stop()
            return

        target = corner_aware_target(
            self.path,
            (robot_x, robot_y),
            float(self.get_parameter("lookahead_distance").value),
            float(self.get_parameter("corner_search_distance").value),
            math.radians(
                float(self.get_parameter("corner_angle_threshold_deg").value)
            ),
            float(self.get_parameter("outward_shift_gain").value),
            float(self.get_parameter("outward_shift_max").value),
        )
        dx, dy = target[0] - robot_x, target[1] - robot_y
        target_angle = wrap_angle(math.atan2(dy, dx) - robot_yaw)
        self._last_target_angle = target_angle
        try:
            histogram, observed, scan_angles, scan_ranges = self._histogram()
        except TransformException as error:
            self.get_logger().warning(
                f"Controller waiting for scan TF: {error}",
                throttle_duration_sec=2.0,
            )
            self._publish_stop()
            return
        clearance_reference = 0.9
        minimum_valley_angle = 2.0 * math.asin(
            min(0.95, float(self.get_parameter("robot_width").value) / (2.0 * clearance_reference))
        )
        minimum_valley_width = max(
            2, int(math.ceil(minimum_valley_angle / self.sector_width))
        )
        margin = max(
            1,
            int(
                math.ceil(
                    math.radians(
                        float(
                            self.get_parameter(
                                "valley_safety_margin_deg"
                            ).value
                        )
                    )
                    / self.sector_width
                )
            ),
        )
        previous_heading = (
            target_angle
            if self.previous_heading_global is None
            else wrap_angle(self.previous_heading_global - robot_yaw)
        )
        selected = choose_vfh_heading(
            histogram,
            observed,
            target_angle,
            previous_heading,
            float(self.get_parameter("obstacle_threshold").value),
            self.sector_width,
            minimum_valley_width,
            margin,
            float(self.get_parameter("heading_weight").value),
            float(self.get_parameter("previous_heading_weight").value),
            float(self.get_parameter("clearance_weight").value),
        )
        if selected is None:
            linear = 0.0
            selected = choose_vfh_recovery_heading(
                histogram,
                observed,
                target_angle,
                previous_heading,
                self.sector_width,
            )
            selected = selected if selected is not None else 0.0
            angular = regulated_angular_velocity(
                selected,
                self.max_angular,
                float(
                    self.get_parameter("angular_proportional_gain").value
                ),
                math.radians(
                    float(self.get_parameter("heading_deadband_deg").value)
                ),
                self.max_angular_decel,
            )
        else:
            front = scan_ranges[np.abs(scan_angles) <= math.radians(30.0)]
            front_distance = float(np.min(front)) if front.size else 0.0
            linear, angular = fuzzy_velocity(
                front_distance, selected, self.max_linear, self.max_angular
            )
            # Large path-heading errors must be closed directly against the
            # path target.  Using the VFH-selected heading here makes a
            # partial-FOV lidar rotate in small observed-sector increments:
            # it reaches the current valley edge, stops, then selects another
            # edge even though the path target is still far from the nose.
            self.aligning = update_alignment_state(
                self.aligning,
                target_angle,
                math.radians(
                    float(self.get_parameter("align_enter_angle_deg").value)
                ),
                math.radians(
                    float(self.get_parameter("align_exit_angle_deg").value)
                ),
            )
            if self.aligning:
                linear = 0.0
            control_heading = target_angle if self.aligning else selected
            angular = regulated_angular_velocity(
                control_heading,
                self.max_angular,
                float(
                    self.get_parameter("angular_proportional_gain").value
                ),
                math.radians(
                    float(self.get_parameter("heading_deadband_deg").value)
                ),
                self.max_angular_decel,
            )
            if linear > 0.0:
                linear = max(linear, self.min_effective_linear)
        linear, angular = self._rate_limit(linear, angular)
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_publisher.publish(command)
        self.previous_heading_global = wrap_angle(robot_yaw + selected)
        self._publish_markers(
            target,
            selected,
            histogram,
            observed,
            float(self.get_parameter("obstacle_threshold").value),
        )


def main(args=None):
    rclpy.init(args=args)
    node = VfhController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # Isaac's embedded ROS bridge can remove a LaserScan endpoint while
        # rclpy is taking its final message during simulator shutdown.
        pass
    finally:
        if rclpy.ok():
            node.cmd_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
