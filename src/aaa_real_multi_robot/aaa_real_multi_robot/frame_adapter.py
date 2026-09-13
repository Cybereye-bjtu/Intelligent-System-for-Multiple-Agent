import copy
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def prefixed(robot_name, frame):
    frame = str(frame).strip().lstrip("/")
    prefix = robot_name.strip().strip("/")
    if not frame:
        return ""
    if frame == prefix or frame.startswith(prefix + "/"):
        return frame
    return f"{prefix}/{frame}"


def quaternion_yaw(quaternion):
    """Return planar yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def relative_se2(x, y, yaw, origin):
    """Express an absolute planar pose relative to an SE(2) origin."""
    origin_x, origin_y, origin_yaw = origin
    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    relative_yaw = math.atan2(
        math.sin(yaw - origin_yaw), math.cos(yaw - origin_yaw)
    )
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        relative_yaw,
    )


class FrameAdapter(Node):
    """Convert one isolated vehicle's raw scan/odometry into fleet-safe names."""

    def __init__(self):
        super().__init__("frame_adapter")
        defaults = {
            "robot_name": "robot",
            "input_scan_topic": "/scan",
            "input_odom_topic": "/odom",
            "output_scan_topic": "scan",
            "output_odom_topic": "odom",
            "odom_parent_frame": "odom",
            "odom_message_frame": "odom",
            "base_frame": "base_footprint",
            "laser_frame": "laser",
            "intermediate_odom_frame": "",
            "publish_tf": True,
            "publish_static_tf": True,
            "laser_x": 0.0,
            "laser_y": 0.0,
            "laser_z": 0.0,
            "laser_yaw": 0.0,
            "force_planar_odom": True,
            "zero_initial_odom": False,
            "synchronize_scan_to_odom": False,
            "max_scan_odom_sync_age": 0.50,
            "max_odom_age": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.robot_name = str(self.get_parameter("robot_name").value).strip("/")
        output_scan = str(self.get_parameter("output_scan_topic").value)
        output_odom = str(self.get_parameter("output_odom_topic").value)
        self.scan_publisher = self.create_publisher(LaserScan, output_scan, qos_profile_sensor_data)
        self.odom_publisher = self.create_publisher(Odometry, output_odom, qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(DiagnosticArray, "adapter_status", 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.last_scan_time = None
        self.last_odom_time = None
        self.odom_origin = None
        self.latest_odom_stamp = None
        self.last_raw_scan_odom_delta = math.nan
        self.pending_scan = None
        self.scan_sync_deferrals = 0
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        if bool(self.get_parameter("publish_static_tf").value):
            self._publish_static_transforms()
        self.create_timer(0.5, self._publish_status)

    def _frame(self, parameter):
        return prefixed(self.robot_name, self.get_parameter(parameter).value)

    def _scan_callback(self, message):
        output = copy.deepcopy(message)
        output.header.frame_id = self._frame("laser_frame")
        if self.latest_odom_stamp is not None:
            scan_stamp = Time.from_msg(output.header.stamp)
            odom_stamp = Time.from_msg(self.latest_odom_stamp)
            if scan_stamp.nanoseconds > 0 and odom_stamp.nanoseconds > 0:
                self.last_raw_scan_odom_delta = (
                    scan_stamp - odom_stamp
                ).nanoseconds / 1e9
        if bool(self.get_parameter("synchronize_scan_to_odom").value):
            now = self.get_clock().now()
            max_age = float(self.get_parameter("max_scan_odom_sync_age").value)
            odom_fresh = (
                self.latest_odom_stamp is not None
                and self.last_odom_time is not None
                and (now - self.last_odom_time).nanoseconds / 1e9 <= max_age
            )
            if not odom_fresh:
                # Keep only the newest scan. Publishing an old scan without a
                # matching odom TF creates the exact map/laser offset this
                # adapter is intended to prevent.
                self.pending_scan = output
                self.scan_sync_deferrals += 1
                return
            output.header.stamp = copy.deepcopy(self.latest_odom_stamp)
        self._publish_scan(output)

    def _publish_scan(self, output):
        self.scan_publisher.publish(output)
        self.last_scan_time = self.get_clock().now()

    def _odom_callback(self, message):
        output = copy.deepcopy(message)
        raw_yaw = quaternion_yaw(message.pose.pose.orientation)
        if bool(self.get_parameter("zero_initial_odom").value):
            if self.odom_origin is None:
                self.odom_origin = (
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                    raw_yaw,
                )
                self.get_logger().info(
                    "Zeroed odometry at x=%.3f y=%.3f yaw=%.2f deg"
                    % (
                        self.odom_origin[0],
                        self.odom_origin[1],
                        math.degrees(self.odom_origin[2]),
                    )
                )
            relative_x, relative_y, relative_yaw = relative_se2(
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                raw_yaw,
                self.odom_origin,
            )
            output.pose.pose.position.x = relative_x
            output.pose.pose.position.y = relative_y
        else:
            relative_yaw = raw_yaw
        if bool(self.get_parameter("force_planar_odom").value):
            output.pose.pose.position.z = 0.0
            output.twist.twist.linear.z = 0.0
            output.twist.twist.angular.x = 0.0
            output.twist.twist.angular.y = 0.0
            output.pose.pose.orientation.x = 0.0
            output.pose.pose.orientation.y = 0.0
            output.pose.pose.orientation.z = math.sin(relative_yaw / 2.0)
            output.pose.pose.orientation.w = math.cos(relative_yaw / 2.0)
        dynamic_parent = self._frame("odom_message_frame")
        base_frame = self._frame("base_frame")
        output.header.frame_id = dynamic_parent
        output.child_frame_id = base_frame
        self.odom_publisher.publish(output)
        self.last_odom_time = self.get_clock().now()
        self.latest_odom_stamp = copy.deepcopy(output.header.stamp)
        if bool(self.get_parameter("publish_tf").value):
            transform = TransformStamped()
            transform.header = copy.deepcopy(output.header)
            transform.header.frame_id = dynamic_parent
            transform.child_frame_id = base_frame
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = copy.deepcopy(output.pose.pose.orientation)
            self.tf_broadcaster.sendTransform(transform)
        if (
            bool(self.get_parameter("synchronize_scan_to_odom").value)
            and self.pending_scan is not None
        ):
            pending = self.pending_scan
            self.pending_scan = None
            pending.header.stamp = copy.deepcopy(self.latest_odom_stamp)
            self._publish_scan(pending)

    def _publish_static_transforms(self):
        transforms = []
        odom_parent = self._frame("odom_parent_frame")
        odom_message = self._frame("odom_message_frame")
        if odom_parent != odom_message:
            link = TransformStamped()
            link.header.stamp = self.get_clock().now().to_msg()
            link.header.frame_id = odom_parent
            link.child_frame_id = odom_message
            link.transform.rotation.w = 1.0
            transforms.append(link)
        laser = TransformStamped()
        laser.header.stamp = self.get_clock().now().to_msg()
        laser.header.frame_id = self._frame("base_frame")
        laser.child_frame_id = self._frame("laser_frame")
        laser.transform.translation.x = float(self.get_parameter("laser_x").value)
        laser.transform.translation.y = float(self.get_parameter("laser_y").value)
        laser.transform.translation.z = float(self.get_parameter("laser_z").value)
        yaw = float(self.get_parameter("laser_yaw").value)
        laser.transform.rotation.z = math.sin(yaw / 2.0)
        laser.transform.rotation.w = math.cos(yaw / 2.0)
        transforms.append(laser)
        self.static_broadcaster.sendTransform(transforms)

    def _publish_status(self):
        now = self.get_clock().now()
        timeout = float(self.get_parameter("max_odom_age").value)
        scan_age = math.inf if self.last_scan_time is None else (now - self.last_scan_time).nanoseconds / 1e9
        odom_age = math.inf if self.last_odom_time is None else (now - self.last_odom_time).nanoseconds / 1e9
        status = DiagnosticStatus()
        status.name = f"{self.robot_name}/frame_adapter"
        status.hardware_id = self.robot_name
        status.level = DiagnosticStatus.OK if scan_age <= timeout and odom_age <= timeout else DiagnosticStatus.ERROR
        status.message = "telemetry healthy" if status.level == DiagnosticStatus.OK else "scan or odometry stale"
        status.values = [
            KeyValue(key="scan_age_sec", value=f"{scan_age:.3f}"),
            KeyValue(key="odom_age_sec", value=f"{odom_age:.3f}"),
            KeyValue(
                key="initial_odom_zeroed",
                value=str(
                    not bool(self.get_parameter("zero_initial_odom").value)
                    or self.odom_origin is not None
                ),
            ),
            KeyValue(
                key="scan_odom_synchronized",
                value=str(bool(self.get_parameter("synchronize_scan_to_odom").value)),
            ),
            KeyValue(
                key="scan_timestamp_mode",
                value=(
                    "latest_odom"
                    if bool(
                        self.get_parameter(
                            "synchronize_scan_to_odom"
                        ).value
                    )
                    else "source"
                ),
            ),
            KeyValue(
                key="raw_scan_latest_odom_delta_sec",
                value=f"{self.last_raw_scan_odom_delta:.3f}",
            ),
            KeyValue(
                key="scan_sync_deferrals",
                value=str(self.scan_sync_deferrals),
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = now.to_msg()
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FrameAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
