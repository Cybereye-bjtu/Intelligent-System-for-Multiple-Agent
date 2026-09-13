"""Lease-gated transport of centrally planned paths to one robot."""

import json
import math
import os
import ssl
import time

import paho.mqtt.client as mqtt
import rclpy
from aaa_navigation.path_transport import encode_path, transform_xy
from aaa_real_multi_robot_interfaces.msg import NavigationLease
from nav_msgs.msg import Path
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener


class MqttPathSender(Node):
    """Transform site-map paths to vehicle odom and enforce fleet permission."""

    def __init__(self):
        super().__init__("mqtt_path_sender")
        defaults = {
            "broker": "",
            "port": 8883,
            "keepalive_seconds": 60,
            "client_id": "aaa-cloud-path",
            "username_env": "HYZX_MQTT_USERNAME",
            "password_env": "HYZX_MQTT_PASSWORD",
            "ca_file": "/etc/ssl/certs/ca-certificates.crt",
            "tls_insecure": False,
            "path_mqtt_topic": "",
            "path_ros_topic": "global_path",
            "lease_topic": "fleet/lease",
            "emergency_stop_topic": "/swarm/emergency_stop",
            "robot_id": 0,
            "global_frame": "site_map",
            "target_frame": "robot/odom",
            "output_frame": "odom",
            "publish_frequency": 5.0,
            "path_timeout": 0.75,
            "lease_receipt_timeout": 0.50,
            "max_points": 2000,
            "enabled_on_start": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not self.get_parameter("broker").value:
            raise RuntimeError("MQTT broker parameter must not be empty")
        if not self.get_parameter("path_mqtt_topic").value:
            raise RuntimeError("MQTT path topic parameter must not be empty")

        self.enabled = bool(self.get_parameter("enabled_on_start").value)
        self.emergency_stop = True
        self.latest_path = None
        self.path_receipt = None
        self.lease = None
        self.lease_receipt = None
        self.sequence = 0
        self.connected = False
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, str(self.get_parameter("path_ros_topic").value), self._path_callback, qos)
        self.create_subscription(NavigationLease, str(self.get_parameter("lease_topic").value), self._lease_callback, 5)
        self.create_subscription(Bool, str(self.get_parameter("emergency_stop_topic").value), self._stop_callback, qos)
        self.create_service(SetBool, "~/enable", self._enable_callback)

        self.client = self._create_client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        username = os.environ.get(str(self.get_parameter("username_env").value), "")
        password = os.environ.get(str(self.get_parameter("password_env").value), "")
        if not username or not password:
            raise RuntimeError("MQTT credential environment variables are missing")
        self.client.username_pw_set(username, password)
        self.client.tls_set(
            ca_certs=str(self.get_parameter("ca_file").value),
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
        self.client.tls_insecure_set(bool(self.get_parameter("tls_insecure").value))
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(
            str(self.get_parameter("broker").value),
            int(self.get_parameter("port").value),
            max(20, int(self.get_parameter("keepalive_seconds").value)),
        )
        self.client.loop_start()
        frequency = max(1.0, float(self.get_parameter("publish_frequency").value))
        self.create_timer(1.0 / frequency, self._tick, clock=self.steady_clock)

    def _create_client(self):
        client_id = str(self.get_parameter("client_id").value)
        try:
            return mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1, client_id=client_id, clean_session=True)
        except (AttributeError, TypeError):
            return mqtt.Client(client_id=client_id, clean_session=True)

    def _on_connect(self, client, userdata, flags, result_code):
        self.connected = int(result_code) == 0

    def _on_disconnect(self, client, userdata, result_code):
        self.connected = False

    def _path_callback(self, message):
        if message.header.frame_id != str(self.get_parameter("global_frame").value):
            return
        self.latest_path = message
        self.path_receipt = self.steady_clock.now()

    def _lease_callback(self, message):
        if int(message.robot_id) != int(self.get_parameter("robot_id").value):
            return
        if message.header.frame_id != str(self.get_parameter("global_frame").value):
            return
        self.lease = message
        self.lease_receipt = self.steady_clock.now()

    def _stop_callback(self, message):
        self.emergency_stop = bool(message.data)

    def _enable_callback(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = "path sender enabled" if self.enabled else "path sender disabled"
        if not self.enabled:
            self._publish([])
        return response

    def _fresh(self, receipt, timeout):
        return receipt is not None and self.steady_clock.now() - receipt <= Duration(seconds=timeout)

    def _permitted(self):
        if not self.enabled or self.emergency_stop or self.lease is None:
            return False
        if not self._fresh(self.lease_receipt, float(self.get_parameter("lease_receipt_timeout").value)):
            return False
        if not self.lease.permit_motion or self.get_clock().now() >= Time.from_msg(self.lease.valid_until):
            return False
        return self._fresh(self.path_receipt, float(self.get_parameter("path_timeout").value))

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    def _points_in_odom(self):
        transform = self.tf_buffer.lookup_transform(
            str(self.get_parameter("target_frame").value),
            str(self.get_parameter("global_frame").value),
            Time(),
            timeout=Duration(seconds=0.08),
        )
        translation = transform.transform.translation
        yaw = self._yaw(transform.transform.rotation)
        return [
            transform_xy(pose.pose.position.x, pose.pose.position.y, translation.x, translation.y, yaw)
            for pose in self.latest_path.poses
        ]

    def _publish(self, points):
        if not self.connected:
            return
        self.sequence += 1
        payload = encode_path(
            points,
            str(self.get_parameter("output_frame").value),
            self.sequence,
            time.time(),
            int(self.get_parameter("max_points").value),
        )
        self.client.publish(str(self.get_parameter("path_mqtt_topic").value), json.dumps(payload, separators=(",", ":")), qos=1, retain=False)

    def _tick(self):
        if not self._permitted():
            self._publish([])
            return
        try:
            self._publish(self._points_in_odom())
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f"Path transport stopped: {error}", throttle_duration_sec=2.0)
            self._publish([])

    def close(self):
        self.enabled = False
        self._publish([])
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    rclpy.init(args=args)
    node = MqttPathSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
