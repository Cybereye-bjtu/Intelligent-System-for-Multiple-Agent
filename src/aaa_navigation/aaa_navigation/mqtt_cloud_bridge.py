"""Bridge MQTT robot telemetry and bounded commands to native ROS 2."""

import json
import math
import os
import queue
import ssl
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .mqtt_codec import (
    compact_vector,
    consistent_scan_angle_max,
    EnvelopeError,
    decode_envelope,
    decode_stamp,
    finite_float,
    finite_ranges,
    fixed_length_floats,
    normalize_frame,
    resample_scan_ranges,
    scan_increment_from_endpoints,
    simple_twist_command,
    twist_publish_command,
)
class MqttCloudBridge(Node):
    """Restore MQTT telemetry to ROS and send bounded cloud commands."""

    def __init__(self):
        super().__init__("mqtt_cloud_bridge")
        defaults = {
            "broker": "",
            "port": 8883,
            "keepalive_seconds": 60,
            "client_id": "aaa-jetson003-cloud",
            "username_env": "HYZX_MQTT_USERNAME",
            "password_env": "HYZX_MQTT_PASSWORD",
            "ca_file": "/etc/ssl/certs/ca-certificates.crt",
            "tls_insecure": False,
            "scan_mqtt_topic": "robot/jetson003/telemetry/scan",
            "odom_mqtt_topic": "robot/jetson003/telemetry/odom",
            "tf_mqtt_topic": "robot/jetson003/telemetry/tf",
            "tf_static_mqtt_topic": "robot/jetson003/telemetry/tf_static",
            "status_mqtt_topic": "robot/jetson003/telemetry/status",
            "command_mqtt_topic": "edge/jetson003/car/cmd_vel",
            "command_payload_format": "simple_twist",
            "scan_ros_topic": "/scan",
            "odom_ros_topic": "/odom",
            "command_ros_topic": "/cmd_vel_cloud",
            "robot_status_ros_topic": "/robot/status",
            "hardware_id": "jetson003",
            "publish_command_control": True,
            "command_frequency": 10.0,
            "command_timeout": 0.35,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.30,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "laser_frame": "laser_link",
            "strip_leading_slashes": True,
            "frame_aliases": [
                "base_footprint:=base_link",
                "lidar_frame:=laser_link",
            ],
            "restamp_telemetry": False,
            "use_envelope_timestamp": False,
            "max_envelope_age": 1.50,
            "max_queue_age": 0.50,
            "compact_range_scale": 0.001,
            "compact_scan_time": 0.11,
            "compact_scan_beam_count": 505,
            "force_planar_odom": True,
            "publish_odom_tf": True,
            "drop_tf_pairs": [
                "map->odom",
                "odom->base_link",
                "base_link->laser_link",
            ],
            "publish_configured_laser_tf": False,
            "publish_configured_odom_link_tf": False,
            "odom_parent_frame": "odom",
            "laser_x": 0.0,
            "laser_y": 0.0,
            "laser_z": 0.14,
            "laser_yaw": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.scan_topic = str(
            self.get_parameter("scan_mqtt_topic").value
        )
        self.odom_topic = str(
            self.get_parameter("odom_mqtt_topic").value
        )
        self.tf_topic = str(self.get_parameter("tf_mqtt_topic").value)
        self.tf_static_topic = str(
            self.get_parameter("tf_static_mqtt_topic").value
        )
        self.robot_status_topic = str(
            self.get_parameter("status_mqtt_topic").value
        )
        self.telemetry_topics = {
            self.scan_topic: "scan",
            self.odom_topic: "odom",
            self.tf_topic: "tf",
            self.tf_static_topic: "tf_static",
            self.robot_status_topic: "status",
        }
        self.drop_tf_pairs = {
            str(pair).replace(" ", "").lstrip("/")
            for pair in self.get_parameter("drop_tf_pairs").value
        }
        self.frame_aliases = {}
        for entry in self.get_parameter("frame_aliases").value:
            source, separator, target = str(entry).partition(":=")
            if separator and source and target:
                self.frame_aliases[
                    normalize_frame(source)
                ] = normalize_frame(target)
        self.queue = queue.Queue(maxsize=200)
        self.last_sequences = {}
        self.last_receipts = {}
        self.last_published = {}
        self.received_counts = {kind: 0 for kind in self.telemetry_topics.values()}
        self.published_counts = {kind: 0 for kind in self.telemetry_topics.values()}
        self.stale_drop_counts = {kind: 0 for kind in self.telemetry_topics.values()}
        self.superseded_counts = {kind: 0 for kind in self.telemetry_topics.values()}
        self.last_envelope_ages = {}
        self.mqtt_connected = False
        self.shutting_down = False
        self.command_enabled = False
        self.latest_command = TwistStamped()
        self.command_receipt_time = None
        self.lock = threading.Lock()

        self.scan_publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("scan_ros_topic").value),
            qos_profile_sensor_data,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_ros_topic").value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/mqtt_bridge/status", 1
        )
        self.robot_status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("robot_status_ros_topic").value),
            1,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.command_subscription = self.create_subscription(
            TwistStamped,
            str(self.get_parameter("command_ros_topic").value),
            self.command_callback,
            1,
        )
        self.enable_service = self.create_service(
            SetBool, "~/enable_commands", self.enable_callback
        )

        self.client = self._create_client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self._configure_mqtt_security()
        self._connect()

        self.telemetry_timer = self.create_timer(0.01, self.drain_queue)
        frequency = max(
            1.0, float(self.get_parameter("command_frequency").value)
        )
        self.command_timer = self.create_timer(
            1.0 / frequency, self.publish_command
        )
        self.status_timer = self.create_timer(0.5, self.publish_status)
        if bool(
            self.get_parameter("publish_configured_laser_tf").value
        ) or bool(
            self.get_parameter(
                "publish_configured_odom_link_tf"
            ).value
        ):
            self.publish_configured_static_tfs()
            self.configured_static_tf_timer = self.create_timer(
                1.0, self.publish_configured_static_tfs
            )
        self.get_logger().warning(
            "MQTT chassis commands are DISABLED. Call "
            "/mqtt_cloud_bridge/enable_commands only after telemetry, "
            "mapping and wheel-off-ground checks pass."
        )

    def _create_client(self):
        client_id = str(self.get_parameter("client_id").value)
        try:
            return mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=client_id,
                clean_session=True,
            )
        except (AttributeError, TypeError):
            return mqtt.Client(client_id=client_id, clean_session=True)

    def _configure_mqtt_security(self):
        username = os.environ.get(
            str(self.get_parameter("username_env").value), ""
        )
        password = os.environ.get(
            str(self.get_parameter("password_env").value), ""
        )
        if not username or not password:
            raise RuntimeError(
                "MQTT credentials are missing; set the configured "
                "username/password environment variables"
            )
        self.client.username_pw_set(username, password)
        ca_file = str(self.get_parameter("ca_file").value)
        self.client.tls_set(
            ca_certs=ca_file,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
        insecure = bool(self.get_parameter("tls_insecure").value)
        self.client.tls_insecure_set(insecure)
        if insecure:
            self.get_logger().warning(
                "MQTT TLS hostname verification is disabled"
            )

    def _connect(self):
        broker = str(self.get_parameter("broker").value)
        if not broker:
            raise RuntimeError("MQTT broker parameter must not be empty")
        port = int(self.get_parameter("port").value)
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        keepalive = max(
            20, int(self.get_parameter("keepalive_seconds").value)
        )
        self.client.connect_async(broker, port, keepalive=keepalive)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, result_code):
        self.mqtt_connected = result_code == 0
        self.command_enabled = False
        if not self.mqtt_connected:
            self.get_logger().error(
                f"MQTT connection rejected with code {result_code}"
            )
            return
        for topic in self.telemetry_topics:
            client.subscribe(topic, qos=0)
        self.get_logger().info("MQTT connected and telemetry subscribed")

    def on_disconnect(self, client, userdata, result_code):
        self.mqtt_connected = False
        self.command_enabled = False
        if self.shutting_down:
            return
        self.get_logger().error(
            f"MQTT disconnected with code {result_code}; commands disabled"
        )

    def on_message(self, client, userdata, message):
        kind = self.telemetry_topics.get(message.topic)
        if kind is None:
            return
        try:
            sequence, timestamp, source, data = decode_envelope(
                message.payload
            )
        except EnvelopeError as error:
            self.get_logger().warning(
                f"Rejected {kind} telemetry: {error}",
                throttle_duration_sec=2.0,
            )
            return
        now = time.monotonic()
        self.received_counts[kind] += 1
        envelope_age = time.time() - timestamp
        # Ignore age checks if edge and cloud wall clocks are clearly not in
        # the same epoch.  The sequence and local queue checks still apply.
        if abs(envelope_age) < 86400.0:
            self.last_envelope_ages[kind] = envelope_age
            max_age = float(self.get_parameter("max_envelope_age").value)
            # A retained tf_static message can be hours old and still be the
            # authoritative immutable vehicle geometry.
            if (
                kind != "tf_static"
                and max_age > 0.0
                and envelope_age > max_age
            ):
                self.stale_drop_counts[kind] += 1
                return
        last_sequence = self.last_sequences.get(kind)
        last_receipt = self.last_receipts.get(kind, 0.0)
        if (
            last_sequence is not None
            and sequence <= last_sequence
            and now - last_receipt < 2.0
        ):
            return
        self.last_sequences[kind] = sequence
        self.last_receipts[kind] = now
        item = (kind, timestamp, source, data, now)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            self.queue.put_nowait(item)

    def _frame(self, value, fallback=""):
        frame = str(value or fallback)
        if bool(self.get_parameter("strip_leading_slashes").value):
            frame = normalize_frame(frame)
        return self.frame_aliases.get(frame, frame)

    def _set_header(
        self,
        output,
        data,
        fallback_frame="",
        envelope_timestamp=None,
    ):
        # ROS 2 serializers normally keep a nested ``header`` block. The
        # Some edge serializers flatten Header to ``stamp`` and ``frame_id``
        # at the message root, so accept both representations.
        header = data.get("header", {})
        if not isinstance(header, dict):
            header = {}
        stamp = header.get(
            "stamp", data.get("stamp", data.get("t", {}))
        )
        use_envelope_timestamp = bool(
            self.get_parameter("use_envelope_timestamp").value
        )
        if (
            use_envelope_timestamp
            and envelope_timestamp is not None
            and math.isfinite(envelope_timestamp)
            and envelope_timestamp > 0.0
        ):
            seconds = int(math.floor(envelope_timestamp))
            nanoseconds = int(
                round((envelope_timestamp - seconds) * 1e9)
            )
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            output.header.stamp.sec = seconds
            output.header.stamp.nanosec = nanoseconds
        elif bool(self.get_parameter("restamp_telemetry").value):
            output.header.stamp = self.get_clock().now().to_msg()
        else:
            seconds, nanoseconds = decode_stamp(stamp)
            if seconds:
                output.header.stamp.sec = seconds
                output.header.stamp.nanosec = nanoseconds
            else:
                output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._frame(
            header.get(
                "frame_id", data.get("frame_id", data.get("f"))
            ),
            fallback_frame,
        )

    @staticmethod
    def _set_vector(output, data):
        output.x = finite_float(data.get("x"))
        output.y = finite_float(data.get("y"))
        output.z = finite_float(data.get("z"))

    @staticmethod
    def _set_quaternion(output, data):
        output.x = finite_float(data.get("x"))
        output.y = finite_float(data.get("y"))
        output.z = finite_float(data.get("z"))
        output.w = finite_float(data.get("w"), 1.0)

    def _decode_scan(self, data, envelope_timestamp=None):
        message = LaserScan()
        compact = "r" in data
        message.angle_min = finite_float(
            data.get("angle_min", data.get("a_min", data.get("amin")))
        )
        message.angle_max = finite_float(
            data.get("angle_max", data.get("a_max", data.get("amax")))
        )
        message.angle_increment = finite_float(
            data.get(
                "angle_increment", data.get("a_inc", data.get("ainc"))
            )
        )
        message.time_increment = finite_float(data.get("time_increment"))
        message.scan_time = finite_float(
            data.get(
                "scan_time",
                self.get_parameter("compact_scan_time").value
                if compact
                else 0.0,
            )
        )
        # edge_agent timestamps its MQTT envelope when the completed scan is
        # forwarded. Use the scan midpoint as the single-pose approximation
        # consumed by Slam Toolbox.
        scan_timestamp = envelope_timestamp
        if (
            scan_timestamp is not None
            and message.scan_time > 0.0
        ):
            scan_timestamp -= 0.5 * message.scan_time
        self._set_header(
            message,
            data,
            str(self.get_parameter("laser_frame").value),
            scan_timestamp,
        )
        message.range_min = finite_float(
            data.get("range_min", data.get("r_min", data.get("rmin"))),
            0.0,
        )
        message.range_max = finite_float(
            data.get("range_max", data.get("r_max", data.get("rmax"))),
            200.0,
        )
        range_scale = (
            float(self.get_parameter("compact_range_scale").value)
            if compact
            else 1.0
        )
        message.ranges = finite_ranges(
            data.get("ranges", data.get("r", [])), range_scale
        )
        if compact and int(
            self.get_parameter("compact_scan_beam_count").value
        ) > 0:
            target_count = int(
                self.get_parameter("compact_scan_beam_count").value
            )
            message.ranges = resample_scan_ranges(
                message.ranges, target_count
            )
        message.intensities = [
            finite_float(value)
            for value in data.get("intensities", [])
        ]
        if (
            not message.ranges
            or message.angle_increment <= 0.0
            or message.range_max <= message.range_min
        ):
            raise EnvelopeError("invalid LaserScan geometry")
        declared_count = int(
            round(
                (message.angle_max - message.angle_min)
                / message.angle_increment
            )
        ) + 1
        actual_count = len(message.ranges)
        if compact:
            recovered_increment = scan_increment_from_endpoints(
                message.angle_min, message.angle_max, actual_count
            )
            if recovered_increment > 0.0:
                message.angle_increment = recovered_increment
            declared_count = actual_count
        if declared_count != actual_count:
            self.get_logger().warning(
                "Normalizing inconsistent LaserScan geometry: "
                f"declared {declared_count} readings, received "
                f"{actual_count}",
                throttle_duration_sec=10.0,
            )
            message.angle_max = consistent_scan_angle_max(
                message.angle_min,
                message.angle_increment,
                actual_count,
            )
        if message.time_increment <= 0.0 and message.scan_time > 0.0:
            message.time_increment = (
                message.scan_time / max(actual_count - 1, 1)
            )
        return message

    def _decode_odom(self, data, envelope_timestamp=None):
        message = Odometry()
        self._set_header(
            message,
            data,
            str(self.get_parameter("odom_frame").value),
            envelope_timestamp,
        )
        message.child_frame_id = self._frame(
            data.get("child_frame_id", data.get("c")),
            str(self.get_parameter("base_frame").value),
        )
        if "p" in data or "o" in data:
            position = compact_vector(data.get("p"), 3)
            orientation = compact_vector(data.get("o"), 4)
            if bool(self.get_parameter("force_planar_odom").value):
                position[2] = 0.0
            (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ) = position
            (
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ) = orientation
            linear = compact_vector(data.get("v"), 3)
            angular = compact_vector(data.get("w"), 3)
            (
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ) = linear
            (
                message.twist.twist.angular.x,
                message.twist.twist.angular.y,
                message.twist.twist.angular.z,
            ) = angular
            return message
        pose_block = data.get("pose", {})
        if not isinstance(pose_block, dict):
            pose_block = {}
        # Accept both nested ROS 2 JSON and the edge agent's flattened pose.
        pose = pose_block.get("pose", pose_block)
        if not isinstance(pose, dict):
            pose = {}
        self._set_vector(
            message.pose.pose.position, pose.get("position", {})
        )
        self._set_quaternion(
            message.pose.pose.orientation, pose.get("orientation", {})
        )
        message.pose.covariance = fixed_length_floats(
            pose_block.get("covariance"), 36
        )
        twist_block = data.get("twist", {})
        if not isinstance(twist_block, dict):
            twist_block = {}
        twist = twist_block.get("twist", twist_block)
        if not isinstance(twist, dict):
            twist = {}
        self._set_vector(
            message.twist.twist.linear, twist.get("linear", {})
        )
        self._set_vector(
            message.twist.twist.angular, twist.get("angular", {})
        )
        message.twist.covariance = fixed_length_floats(
            twist_block.get("covariance"), 36
        )
        return message

    def _decode_transform(self, data, envelope_timestamp=None):
        message = TransformStamped()
        self._set_header(
            message, data, envelope_timestamp=envelope_timestamp
        )
        message.child_frame_id = self._frame(
            data.get("child_frame_id", data.get("c"))
        )
        if any(key in data for key in ("p", "q", "tr", "r")):
            position = compact_vector(data.get("p", data.get("tr")), 3)
            quaternion = compact_vector(data.get("q", data.get("r")), 4)
            (
                message.transform.translation.x,
                message.transform.translation.y,
                message.transform.translation.z,
            ) = position
            (
                message.transform.rotation.x,
                message.transform.rotation.y,
                message.transform.rotation.z,
                message.transform.rotation.w,
            ) = quaternion
            if not message.header.frame_id or not message.child_frame_id:
                raise EnvelopeError("transform frames must not be empty")
            return message
        transform = data.get("transform", {})
        self._set_vector(
            message.transform.translation,
            transform.get("translation", {}),
        )
        self._set_quaternion(
            message.transform.rotation,
            transform.get("rotation", {}),
        )
        if not message.header.frame_id or not message.child_frame_id:
            raise EnvelopeError("transform frames must not be empty")
        return message

    def _drop_transform(self, transform):
        pair = (
            f"{normalize_frame(transform.header.frame_id)}->"
            f"{normalize_frame(transform.child_frame_id)}"
        )
        return pair in self.drop_tf_pairs

    def _decode_robot_status(
        self, data, source, envelope_timestamp=None
    ):
        status = DiagnosticStatus()
        hardware_id = str(self.get_parameter("hardware_id").value)
        status.name = f"{hardware_id}_robot_status"
        status.hardware_id = hardware_id
        compact = "v" in data and "battery_voltage" not in data
        error_code = int(finite_float(data.get("error_code")))
        status.level = (
            DiagnosticStatus.OK
            if error_code == 0
            else DiagnosticStatus.ERROR
        )
        status.message = (
            "OK" if error_code == 0 else f"ERROR_{error_code}"
        )
        status.values = [
            KeyValue(key="source", value=str(source)),
            KeyValue(
                key="battery_voltage",
                value=str(
                    finite_float(
                        data.get("battery_voltage", data.get("v"))
                    )
                ),
            ),
            KeyValue(
                key="charging",
                value=str(bool(data.get("chg", False))),
            ),
            KeyValue(
                key="charging_current",
                value=str(finite_float(data.get("chg_a"))),
            ),
            KeyValue(
                key="linear_velocity",
                value=str(finite_float(data.get("vx"))),
            ),
            KeyValue(
                key="angular_velocity",
                value=str(finite_float(data.get("wz"))),
            ),
            KeyValue(
                key="motion_mode",
                value=str(int(finite_float(data.get("motion_mode")))),
            ),
            KeyValue(
                key="vehicle_state",
                value=str(int(finite_float(data.get("vehicle_state")))),
            ),
            KeyValue(
                key="control_mode",
                value=str(int(finite_float(data.get("control_mode")))),
            ),
            KeyValue(key="error_code", value=str(error_code)),
        ]
        if compact:
            status.message = "OK"
        message = DiagnosticArray()
        self._set_header(
            message, data, envelope_timestamp=envelope_timestamp
        )
        message.status = [status]
        return message

    def drain_queue(self):
        latest = {}
        for _ in range(200):
            try:
                item = self.queue.get_nowait()
            except queue.Empty:
                break
            kind = item[0]
            if kind in latest:
                self.superseded_counts[kind] += 1
            latest[kind] = item
        # Publish transforms and odometry before the scan that consumes them.
        for kind in ("tf_static", "tf", "odom", "scan", "status"):
            if kind not in latest:
                continue
            _, timestamp, source, data, receipt_time = latest[kind]
            queue_age = time.monotonic() - receipt_time
            max_queue_age = float(self.get_parameter("max_queue_age").value)
            if max_queue_age > 0.0 and queue_age > max_queue_age:
                self.stale_drop_counts[kind] += 1
                continue
            try:
                if kind == "scan":
                    self.scan_publisher.publish(
                        self._decode_scan(data, timestamp)
                    )
                elif kind == "odom":
                    odom = self._decode_odom(data, timestamp)
                    self.odom_publisher.publish(odom)
                    if bool(self.get_parameter("publish_odom_tf").value):
                        transform = TransformStamped()
                        transform.header = odom.header
                        transform.child_frame_id = odom.child_frame_id
                        transform.transform.translation.x = (
                            odom.pose.pose.position.x
                        )
                        transform.transform.translation.y = (
                            odom.pose.pose.position.y
                        )
                        transform.transform.translation.z = (
                            odom.pose.pose.position.z
                        )
                        transform.transform.rotation = (
                            odom.pose.pose.orientation
                        )
                        self.tf_broadcaster.sendTransform(transform)
                elif kind in ("tf", "tf_static"):
                    transforms = []
                    if isinstance(data, list):
                        items = data
                        common_timestamp = None
                    else:
                        items = data.get("transforms", data.get("tx", []))
                        common_timestamp = data.get("t")
                    for item in items:
                        try:
                            transform_data = item
                            if (
                                common_timestamp is not None
                                and "t" not in item
                            ):
                                transform_data = dict(item)
                                transform_data["t"] = common_timestamp
                            transform = self._decode_transform(
                                transform_data, timestamp
                            )
                        except (
                            EnvelopeError,
                            AttributeError,
                            TypeError,
                        ):
                            continue
                        if not self._drop_transform(transform):
                            transforms.append(transform)
                    if not transforms:
                        continue
                    if kind == "tf":
                        self.tf_broadcaster.sendTransform(transforms)
                    else:
                        self.static_tf_broadcaster.sendTransform(
                            transforms
                        )
                elif kind == "status":
                    self.robot_status_publisher.publish(
                        self._decode_robot_status(
                            data, source, timestamp
                        )
                    )
                self.published_counts[kind] += 1
                self.last_published[kind] = time.monotonic()
            except (EnvelopeError, AttributeError, TypeError) as error:
                self.get_logger().warning(
                    f"Failed to restore {kind} telemetry: {error}",
                    throttle_duration_sec=2.0,
                )

    def publish_configured_static_tfs(self):
        transforms = []
        if bool(
            self.get_parameter("publish_configured_laser_tf").value
        ):
            laser = TransformStamped()
            laser.header.stamp = self.get_clock().now().to_msg()
            laser.header.frame_id = str(
                self.get_parameter("base_frame").value
            )
            laser.child_frame_id = str(
                self.get_parameter("laser_frame").value
            )
            laser.transform.translation.x = float(
                self.get_parameter("laser_x").value
            )
            laser.transform.translation.y = float(
                self.get_parameter("laser_y").value
            )
            laser.transform.translation.z = float(
                self.get_parameter("laser_z").value
            )
            yaw = float(self.get_parameter("laser_yaw").value)
            laser.transform.rotation.z = math.sin(yaw / 2.0)
            laser.transform.rotation.w = math.cos(yaw / 2.0)
            transforms.append(laser)
        if bool(
            self.get_parameter(
                "publish_configured_odom_link_tf"
            ).value
        ):
            odom_link = TransformStamped()
            odom_link.header.stamp = self.get_clock().now().to_msg()
            odom_link.header.frame_id = str(
                self.get_parameter("odom_parent_frame").value
            )
            odom_link.child_frame_id = str(
                self.get_parameter("odom_frame").value
            )
            odom_link.transform.rotation.w = 1.0
            transforms.append(odom_link)
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def command_callback(self, message):
        with self.lock:
            self.latest_command = message
            self.command_receipt_time = self.get_clock().now()

    def _command_is_fresh(self):
        if self.command_receipt_time is None:
            return False
        age = (
            self.get_clock().now() - self.command_receipt_time
        ).nanoseconds / 1e9
        return age <= float(self.get_parameter("command_timeout").value)

    def enable_callback(self, request, response):
        if not request.data:
            self.command_enabled = False
            self._publish_stop("cloud commands manually disabled")
            response.success = True
            response.message = "MQTT commands disabled and zeroed"
            return response
        if not self.mqtt_connected:
            response.success = False
            response.message = "cannot enable: MQTT is disconnected"
            return response
        if not self._command_is_fresh():
            response.success = False
            response.message = "cannot enable: ROS cloud command is stale"
            return response
        self.command_enabled = True
        response.success = True
        response.message = "MQTT cloud commands enabled"
        self.get_logger().warning(response.message)
        return response

    def _bounded_command(self):
        output = Twist()
        if not self.command_enabled or not self._command_is_fresh():
            return output
        command = self.latest_command.twist
        if not (
            math.isfinite(command.linear.x)
            and math.isfinite(command.angular.z)
        ):
            return output
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
        return output

    def publish_command(self):
        if not self.mqtt_connected or not self.command_enabled:
            return
        if not self._command_is_fresh():
            self.command_enabled = False
            self._publish_stop("cloud ROS command timed out")
            self.get_logger().error(
                "Cloud command timed out; MQTT commands latched disabled"
            )
            return
        command = self._bounded_command()
        if bool(self.get_parameter("publish_command_control").value):
            self.client.publish(
                str(self.get_parameter("command_mqtt_topic").value),
                json.dumps(self._encode_command(command), separators=(",", ":")),
                qos=1,
                retain=False,
            )

    def _encode_command(self, command):
        payload_format = str(
            self.get_parameter("command_payload_format").value
        ).strip().lower()
        if payload_format == "simple_twist":
            return simple_twist_command(
                command.linear.x, command.angular.z
            )
        if payload_format == "ros_publish":
            return twist_publish_command(
                command.linear.x, command.angular.z
            )
        raise RuntimeError(
            "command_payload_format must be 'simple_twist' or "
            "'ros_publish'"
        )

    def _publish_stop(self, _reason):
        if not self.mqtt_connected:
            return
        if bool(self.get_parameter("publish_command_control").value):
            self.client.publish(
                str(self.get_parameter("command_mqtt_topic").value),
                json.dumps(self._encode_command(Twist()), separators=(",", ":")),
                qos=1,
                retain=False,
            )

    def _age(self, kind):
        receipt = self.last_receipts.get(kind)
        return float("inf") if receipt is None else time.monotonic() - receipt

    def _publish_age(self, kind):
        receipt = self.last_published.get(kind)
        return float("inf") if receipt is None else time.monotonic() - receipt

    def publish_status(self):
        status = DiagnosticStatus()
        status.name = "aaa_mqtt_cloud_bridge"
        status.hardware_id = str(self.get_parameter("hardware_id").value)
        scan_fresh = self._publish_age("scan") < 0.75
        odom_fresh = self._publish_age("odom") < 0.75
        healthy = self.mqtt_connected and scan_fresh and odom_fresh
        status.level = (
            DiagnosticStatus.OK
            if healthy
            else DiagnosticStatus.ERROR
        )
        status.message = "READY" if healthy else "WAITING_FOR_TELEMETRY"
        status.values = [
            KeyValue(key="mqtt_connected", value=str(self.mqtt_connected)),
            KeyValue(key="commands_enabled", value=str(self.command_enabled)),
            KeyValue(
                key="telemetry_timestamp_mode",
                value=(
                    "envelope"
                    if bool(
                        self.get_parameter(
                            "use_envelope_timestamp"
                        ).value
                    )
                    else (
                        "receipt"
                        if bool(
                            self.get_parameter(
                                "restamp_telemetry"
                            ).value
                        )
                        else "message"
                    )
                ),
            ),
            KeyValue(key="scan_age_sec", value=f"{self._age('scan'):.3f}"),
            KeyValue(key="odom_age_sec", value=f"{self._age('odom'):.3f}"),
            KeyValue(
                key="scan_publish_age_sec",
                value=f"{self._publish_age('scan'):.3f}",
            ),
            KeyValue(
                key="odom_publish_age_sec",
                value=f"{self._publish_age('odom'):.3f}",
            ),
            KeyValue(key="queue_depth", value=str(self.queue.qsize())),
            KeyValue(key="tf_age_sec", value=f"{self._age('tf'):.3f}"),
            KeyValue(
                key="tf_static_age_sec",
                value=f"{self._age('tf_static'):.3f}",
            ),
            KeyValue(
                key="robot_status_age_sec",
                value=f"{self._age('status'):.3f}",
            ),
        ]
        for kind in ("scan", "odom", "tf", "tf_static", "status"):
            status.values.extend([
                KeyValue(
                    key=f"{kind}_received",
                    value=str(self.received_counts[kind]),
                ),
                KeyValue(
                    key=f"{kind}_published",
                    value=str(self.published_counts[kind]),
                ),
                KeyValue(
                    key=f"{kind}_stale_dropped",
                    value=str(self.stale_drop_counts[kind]),
                ),
                KeyValue(
                    key=f"{kind}_superseded",
                    value=str(self.superseded_counts[kind]),
                ),
                KeyValue(
                    key=f"{kind}_envelope_age_sec",
                    value=f"{self.last_envelope_ages.get(kind, float('nan')):.3f}",
                ),
            ])
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.status_publisher.publish(message)

    def shutdown(self):
        self.shutting_down = True
        was_enabled = self.command_enabled
        self.command_enabled = False
        if was_enabled:
            self._publish_stop("cloud MQTT bridge shutdown")
            time.sleep(0.15)
        self.client.loop_stop()
        self.client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = MqttCloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
