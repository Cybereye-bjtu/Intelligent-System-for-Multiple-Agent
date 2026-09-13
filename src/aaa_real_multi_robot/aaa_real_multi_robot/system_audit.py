import math
import time

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.msg import LocalizedLaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def diagnostic_level_value(level):
    """Normalize uint8 fields across ROS Python generator representations."""
    if isinstance(level, (bytes, bytearray)):
        if len(level) != 1:
            raise ValueError("diagnostic level must contain exactly one byte")
        return level[0]
    return int(level)


class SystemAudit(Node):
    def __init__(self):
        super().__init__("system_audit")
        self.declare_parameter("global_frame", "site_map")
        self.declare_parameter("required_topics", [
            "/hyzx001/scan", "/hyzx001/odom", "/jetson003/scan", "/jetson003/odom",
            "/localized_scan", "/swarm/map",
        ])
        self.declare_parameter("base_frames", ["hyzx001/base_footprint", "jetson003/base_link"])
        self.declare_parameter("message_timeout", 5.0)
        self.declare_parameter("discovery_timeout", 10.0)
        self.declare_parameter("tf_timeout", 0.5)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.received = {}
        self.scan_health = {}
        self.localized_scan_sources = set()
        self.bad_diagnostics = {}
        self._data_subscriptions = []
        for robot in ("hyzx001", "jetson003"):
            scan_topic = f"/{robot}/scan"
            odom_topic = f"/{robot}/odom"
            self._data_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    scan_topic,
                    lambda message, topic=scan_topic: self._scan_callback(
                        topic, message
                    ),
                    qos_profile_sensor_data,
                )
            )
            self._data_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda message, topic=odom_topic: self._record(topic),
                    qos_profile_sensor_data,
                )
            )
            self._data_subscriptions.append(
                self.create_subscription(
                    DiagnosticArray,
                    f"/{robot}/adapter_status",
                    self._diagnostic_callback,
                    1,
                )
            )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._data_subscriptions.append(
            self.create_subscription(
                OccupancyGrid,
                "/swarm/map",
                lambda message: self._record("/swarm/map"),
                map_qos,
            )
        )
        self._data_subscriptions.append(
            self.create_subscription(
                LocalizedLaserScan,
                "/localized_scan",
                self._localized_scan_callback,
                10,
            )
        )

    def _record(self, topic):
        self.received[topic] = time.monotonic()

    def _scan_callback(self, topic, message):
        self._record(topic)
        valid = sum(
            1
            for value in message.ranges
            if math.isfinite(value)
            and value >= max(0.01, float(message.range_min))
            and (
                float(message.range_max) <= 0.0
                or value <= float(message.range_max)
            )
        )
        self.scan_health[topic] = (valid, len(message.ranges))

    def _localized_scan_callback(self, message):
        self._record("/localized_scan")
        source = message.scan.header.frame_id.lstrip("/").split("/", 1)[0]
        if source:
            self.localized_scan_sources.add(source)

    def _diagnostic_callback(self, message):
        for status in message.status:
            if diagnostic_level_value(status.level) >= diagnostic_level_value(
                DiagnosticStatus.ERROR
            ):
                self.bad_diagnostics[status.name] = status.message
            else:
                self.bad_diagnostics.pop(status.name, None)

    def _lifecycle_failures(self):
        failures = []
        full_node_names = {
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        }
        for full_name in (
            "/hyzx001/slam_toolbox",
            "/jetson003/slam_toolbox",
            "/hyzx001/amcl",
            "/jetson003/amcl",
            "/swarm_map_server",
        ):
            if full_name not in full_node_names:
                continue
            client = self.create_client(GetState, f"{full_name}/get_state")
            if not client.wait_for_service(timeout_sec=0.2):
                failures.append(f"{full_name}=no lifecycle service")
                continue
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
            result = future.result()
            if result is None or result.current_state.label != "active":
                label = "unknown" if result is None else result.current_state.label
                failures.append(f"{full_name}={label}")
        return failures

    def run(self):
        required_topics = list(self.get_parameter("required_topics").value)
        freshness_topics = [
            "/hyzx001/scan", "/hyzx001/odom", "/jetson003/scan", "/jetson003/odom",
            "/swarm/map",
        ]
        deadline = time.monotonic() + float(
            self.get_parameter("discovery_timeout").value
        )
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            topics = {name for name, _ in self.get_topic_names_and_types()}
            graph_ready = all(
                name in topics and self.get_publishers_info_by_topic(name)
                for name in required_topics
            )
            data_ready = all(name in self.received for name in freshness_topics)
            if graph_ready and data_ready and {"hyzx001", "jetson003"}.issubset(
                self.localized_scan_sources
            ):
                break
        topics = {name for name, _ in self.get_topic_names_and_types()}
        missing_topics = [name for name in required_topics if name not in topics]
        no_publishers = [
            name for name in required_topics
            if name in topics and not self.get_publishers_info_by_topic(name)
        ]
        message_timeout = float(self.get_parameter("message_timeout").value)
        now_monotonic = time.monotonic()
        stale_topics = [
            name for name in freshness_topics
            if name not in self.received or now_monotonic - self.received[name] > message_timeout
        ]
        unhealthy_scans = []
        for topic in ("/hyzx001/scan", "/jetson003/scan"):
            valid, total = self.scan_health.get(topic, (0, 0))
            minimum = max(5, int(total * 0.01))
            if total <= 0 or valid < minimum:
                unhealthy_scans.append(
                    f"{topic} ({valid}/{total} valid ranges)"
                )
        missing_frames = []
        global_frame = str(self.get_parameter("global_frame").value)
        for frame in self.get_parameter("base_frames").value:
            try:
                transform = self.tf_buffer.lookup_transform(global_frame, str(frame), Time(), timeout=Duration(seconds=0.1))
                stamp = Time.from_msg(transform.header.stamp)
                age = (self.get_clock().now() - stamp).nanoseconds / 1e9
                if stamp.nanoseconds <= 0 or age < -0.1 or age > float(self.get_parameter("tf_timeout").value):
                    missing_frames.append(f"{frame} (stale {age:.3f}s)")
            except TransformException:
                missing_frames.append(str(frame))
        bare_frames = {"map", "odom", "odom_combined", "base_footprint", "base_link", "laser", "laser_link", "lidar_frame"}
        parsed_frames = set((yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}).keys())
        found_bare = sorted(bare_frames.intersection(parsed_frames))
        if missing_topics:
            self.get_logger().error("Missing topics: " + ", ".join(missing_topics))
        if no_publishers:
            self.get_logger().error("Topics without publishers: " + ", ".join(no_publishers))
        if stale_topics:
            self.get_logger().error("Topics without fresh messages: " + ", ".join(stale_topics))
        if unhealthy_scans:
            self.get_logger().error(
                "Laser scans without usable ranges: "
                + ", ".join(unhealthy_scans)
            )
        localized_scan_publishers = {
            info.node_namespace.strip("/")
            for info in self.get_publishers_info_by_topic("/localized_scan")
        }
        missing_scan_publishers = sorted(
            {"hyzx001", "jetson003"}.difference(localized_scan_publishers)
        )
        if missing_scan_publishers:
            self.get_logger().error(
                "Robots without a localized-scan publisher: "
                + ", ".join(missing_scan_publishers)
            )
        if missing_frames:
            self.get_logger().error("Missing transforms from site_map: " + ", ".join(missing_frames))
        if found_bare:
            self.get_logger().error("Unsafe bare frame IDs present: " + ", ".join(found_bare))
        if self.bad_diagnostics:
            self.get_logger().error(
                "Unhealthy adapters: "
                + ", ".join(f"{name}={detail}" for name, detail in self.bad_diagnostics.items())
            )
        lifecycle_failures = self._lifecycle_failures()
        if lifecycle_failures:
            self.get_logger().error("Inactive lifecycle nodes: " + ", ".join(lifecycle_failures))
        if not any((missing_topics, no_publishers, stale_topics, unhealthy_scans, missing_scan_publishers, missing_frames, found_bare, self.bad_diagnostics, lifecycle_failures)):
            self.get_logger().info(
                "PASS: both decentralized SLAM publishers, /swarm/map, and isolated TF are healthy"
            )
            return 0
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = SystemAudit()
    result = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(result)
