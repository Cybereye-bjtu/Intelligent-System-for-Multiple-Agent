import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, UInt64
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .se2 import distance, wrap


def transform_to_se2(message):
    rotation = message.transform.rotation
    values = (
        message.transform.translation.x,
        message.transform.translation.y,
        message.transform.translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("alignment contains non-finite values")
    norm = math.sqrt(rotation.x ** 2 + rotation.y ** 2 + rotation.z ** 2 + rotation.w ** 2)
    if abs(norm - 1.0) > 0.01:
        raise ValueError("alignment quaternion is not normalized")
    roll = math.atan2(
        2.0 * (rotation.w * rotation.x + rotation.y * rotation.z),
        1.0 - 2.0 * (rotation.x * rotation.x + rotation.y * rotation.y),
    )
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (rotation.w * rotation.y - rotation.z * rotation.x))))
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return (
        float(message.transform.translation.x),
        float(message.transform.translation.y),
        wrap(yaw),
    ), float(message.transform.translation.z), roll, pitch


class AlignmentManager(Node):
    """Own site_map->robot/map and admit only stable external alignment updates."""

    def __init__(self):
        super().__init__("alignment_manager")
        self.declare_parameter("global_frame", "site_map")
        self.declare_parameter("robot_names", ["hyzx001", "jetson003"])
        self.declare_parameter("initial_poses_xy_yaw_deg", [0.0, 0.0, 0.0, 4.0, 0.0, 0.0])
        self.declare_parameter("candidate_topic", "/swarm/alignment_candidate")
        self.declare_parameter("updates_enabled", False)
        self.declare_parameter("confirmations_required", 5)
        self.declare_parameter("candidate_translation_tolerance", 0.05)
        self.declare_parameter("candidate_rotation_tolerance_deg", 1.0)
        self.declare_parameter("max_update_translation", 0.20)
        self.declare_parameter("max_update_rotation_deg", 3.0)
        self.declare_parameter("publish_frequency", 5.0)
        self.declare_parameter("candidate_max_age", 1.0)
        self.declare_parameter("max_candidate_z", 0.10)
        self.declare_parameter("max_candidate_tilt_deg", 3.0)
        self.declare_parameter("map_timeout", 5.0)
        self.declare_parameter("anchor_robot", "hyzx001")
        self.declare_parameter("max_calibration_translation", 10.0)
        self.declare_parameter("max_calibration_rotation_deg", 180.0)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robots = [str(name).strip("/") for name in self.get_parameter("robot_names").value]
        raw = [float(value) for value in self.get_parameter("initial_poses_xy_yaw_deg").value]
        if len(raw) != 3 * len(self.robots):
            raise ValueError("initial_poses_xy_yaw_deg must contain x,y,yaw for every robot")
        self.alignments = {
            name: (raw[3 * index], raw[3 * index + 1], math.radians(raw[3 * index + 2]))
            for index, name in enumerate(self.robots)
        }
        self.updates_enabled = bool(self.get_parameter("updates_enabled").value)
        self.calibration_mode = False
        self.emergency_stop = True
        self.pending = {}
        self.map_receipts = {}
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.map_epoch = 1
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.epoch_publisher = self.create_publisher(UInt64, "/swarm/map_epoch", qos)
        self.status_publisher = self.create_publisher(DiagnosticArray, "/swarm/map_alignment", qos)
        self.broadcaster = TransformBroadcaster(self)
        self._candidate_subscription = self.create_subscription(
            TransformStamped,
            str(self.get_parameter("candidate_topic").value),
            self._candidate_callback,
            10,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscriptions = [
            self.create_subscription(
                OccupancyGrid,
                f"/{robot}/map",
                lambda message, name=robot: self._map_callback(name, message),
                map_qos,
            )
            for robot in self.robots
        ]
        self._stop_subscription = self.create_subscription(
            Bool,
            "/swarm/emergency_stop",
            self._stop_callback,
            qos,
        )
        self.create_service(SetBool, "~/enable_updates", self._enable_callback)
        self.create_service(
            SetBool, "~/enable_calibration", self._calibration_callback
        )
        frequency = max(1.0, float(self.get_parameter("publish_frequency").value))
        self.create_timer(1.0 / frequency, self._publish)

    def _robot_from_child(self, child):
        normalized = str(child).strip().lstrip("/")
        for robot in self.robots:
            if normalized == f"{robot}/map":
                return robot
        return None

    def _candidate_callback(self, message):
        robot = self._robot_from_child(message.child_frame_id)
        if not self.updates_enabled or robot is None or message.header.frame_id != self.global_frame:
            return
        if self.calibration_mode and robot == str(
            self.get_parameter("anchor_robot").value
        ):
            return
        stamp = Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if stamp.nanoseconds <= 0 or age < -0.10 or age > float(self.get_parameter("candidate_max_age").value):
            return
        try:
            candidate, z_value, roll, pitch = transform_to_se2(message)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        max_tilt = math.radians(float(self.get_parameter("max_candidate_tilt_deg").value))
        if abs(z_value) > float(self.get_parameter("max_candidate_z").value) or abs(roll) > max_tilt or abs(pitch) > max_tilt:
            self.get_logger().error("Rejected non-planar alignment candidate", throttle_duration_sec=2.0)
            return
        translation, rotation = distance(candidate, self.alignments[robot])
        max_translation = float(
            self.get_parameter(
                "max_calibration_translation"
                if self.calibration_mode
                else "max_update_translation"
            ).value
        )
        max_rotation = math.radians(
            float(
                self.get_parameter(
                    "max_calibration_rotation_deg"
                    if self.calibration_mode
                    else "max_update_rotation_deg"
                ).value
            )
        )
        if translation > max_translation or rotation > max_rotation:
            self.get_logger().error(f"Rejected unsafe alignment jump for {robot}: {translation:.3f} m, {math.degrees(rotation):.2f} deg")
            self.pending.pop(robot, None)
            return
        previous, count = self.pending.get(robot, (candidate, 0))
        candidate_delta = distance(candidate, previous)
        stable = (
            candidate_delta[0] <= float(self.get_parameter("candidate_translation_tolerance").value)
            and candidate_delta[1] <= math.radians(float(self.get_parameter("candidate_rotation_tolerance_deg").value))
        )
        count = count + 1 if stable else 1
        self.pending[robot] = (candidate, count)
        if count >= int(self.get_parameter("confirmations_required").value):
            self.alignments[robot] = candidate
            self.pending.pop(robot, None)
            self.map_epoch += 1
            self.get_logger().warning(f"Accepted alignment update for {robot}; map epoch is now {self.map_epoch}")
            if self.calibration_mode:
                self.calibration_mode = False
                self.updates_enabled = False
                self.get_logger().warning(
                    "One-shot calibration accepted; alignment updates are locked"
                )

    def _map_callback(self, robot, message):
        if message.header.frame_id == f"{robot}/map" and message.info.width and message.info.height:
            self.map_receipts[robot] = self.steady_clock.now()

    def _stop_callback(self, message):
        self.emergency_stop = bool(message.data)

    def _enable_callback(self, request, response):
        self.calibration_mode = False
        self.updates_enabled = bool(request.data)
        if not self.updates_enabled:
            self.pending.clear()
        response.success = True
        response.message = "alignment updates enabled" if self.updates_enabled else "alignment updates locked"
        return response

    def _calibration_callback(self, request, response):
        if not request.data:
            self.calibration_mode = False
            self.updates_enabled = False
            self.pending.clear()
            response.success = True
            response.message = "one-shot calibration cancelled and locked"
            return response
        timeout = Duration(seconds=float(self.get_parameter("map_timeout").value))
        now = self.steady_clock.now()
        maps_fresh = all(
            robot in self.map_receipts
            and now - self.map_receipts[robot] <= timeout
            for robot in self.robots
        )
        if not self.emergency_stop or not maps_fresh:
            response.success = False
            response.message = (
                "calibration requires active emergency stop and two fresh maps"
            )
            return response
        self.pending.clear()
        self.calibration_mode = True
        self.updates_enabled = True
        response.success = True
        response.message = (
            "one-shot calibration armed for the non-anchor robot; "
            "it will auto-lock after acceptance"
        )
        self.get_logger().warning(response.message)
        return response

    def _publish(self):
        now = self.get_clock().now().to_msg()
        transforms = []
        for robot, (x, y, yaw) in self.alignments.items():
            message = TransformStamped()
            message.header.stamp = now
            message.header.frame_id = self.global_frame
            message.child_frame_id = f"{robot}/map"
            message.transform.translation.x = x
            message.transform.translation.y = y
            message.transform.rotation.z = math.sin(yaw / 2.0)
            message.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(message)
        self.broadcaster.sendTransform(transforms)
        epoch = UInt64()
        epoch.data = self.map_epoch
        self.epoch_publisher.publish(epoch)
        item = DiagnosticStatus()
        item.name = "swarm/map_alignment"
        item.hardware_id = "fleet"
        timeout = Duration(seconds=float(self.get_parameter("map_timeout").value))
        active = [
            robot for robot, receipt in self.map_receipts.items()
            if self.steady_clock.now() - receipt <= timeout
        ]
        missing = [robot for robot in self.robots if robot not in active]
        item.level = DiagnosticStatus.OK if not missing else DiagnosticStatus.WARN
        item.message = f"{len(active)}/{len(self.robots)} maps active; updates {'enabled' if self.updates_enabled else 'locked'}"
        item.values = [
            KeyValue(key="map_epoch", value=str(self.map_epoch)),
            KeyValue(key="active_maps", value=",".join(active)),
            KeyValue(key="missing_maps", value=",".join(missing)),
        ]
        status = DiagnosticArray()
        status.header.stamp = now
        status.status = [item]
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
