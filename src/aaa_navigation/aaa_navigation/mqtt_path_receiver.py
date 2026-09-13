"""Receive a server-planned path without exposing a chassis command topic."""

import json
import os
import queue
import ssl
import time

import paho.mqtt.client as mqtt
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .path_transport import decode_path


class MqttPathReceiver(Node):
    """Publish only fresh, ordered paths; timeout and disconnect clear the path."""

    def __init__(self):
        super().__init__("mqtt_path_receiver")
        defaults = {
            "broker": "",
            "port": 8883,
            "keepalive_seconds": 60,
            "client_id": "aaa-edge-path",
            "username_env": "HYZX_MQTT_USERNAME",
            "password_env": "HYZX_MQTT_PASSWORD",
            "ca_file": "/etc/ssl/certs/ca-certificates.crt",
            "tls_insecure": False,
            "path_mqtt_topic": "",
            "path_ros_topic": "/aaa/global_path_local",
            "output_frame": "odom",
            "path_timeout": 0.80,
            "max_message_age": 1.50,
            "max_points": 2000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not self.get_parameter("broker").value:
            raise RuntimeError("MQTT broker parameter must not be empty")
        if not self.get_parameter("path_mqtt_topic").value:
            raise RuntimeError("MQTT path topic parameter must not be empty")

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter("path_ros_topic").value), qos
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, "/aaa/path_receiver/status", 1
        )
        self.incoming = queue.Queue(maxsize=1)
        self.connected = False
        self.last_sequence = -1
        self.last_path_time = None
        self.path_active = False

        self.client = self._create_client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
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
        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._publish_status)

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

    def _on_connect(self, client, userdata, flags, result_code):
        self.connected = int(result_code) == 0
        if self.connected:
            client.subscribe(str(self.get_parameter("path_mqtt_topic").value), qos=1)

    def _on_disconnect(self, client, userdata, result_code):
        self.connected = False

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            decoded = decode_path(
                payload,
                str(self.get_parameter("output_frame").value),
                int(self.get_parameter("max_points").value),
            )
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
            return
        while not self.incoming.empty():
            try:
                self.incoming.get_nowait()
            except queue.Empty:
                break
        self.incoming.put_nowait(decoded)

    def _empty_path(self):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("output_frame").value)
        self.publisher.publish(message)
        self.path_active = False

    def _publish_path(self, points):
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("output_frame").value)
        for x, y in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.publisher.publish(message)
        self.path_active = len(points) >= 2

    def _tick(self):
        try:
            sequence, timestamp, points = self.incoming.get_nowait()
        except queue.Empty:
            sequence = None
        if sequence is not None and sequence > self.last_sequence:
            age = time.time() - timestamp
            if -0.5 <= age <= float(self.get_parameter("max_message_age").value):
                self.last_sequence = sequence
                self.last_path_time = time.monotonic()
                self._publish_path(points)
        stale = (
            not self.connected
            or self.last_path_time is None
            or time.monotonic() - self.last_path_time
            > float(self.get_parameter("path_timeout").value)
        )
        if stale and self.path_active:
            self._empty_path()

    def _publish_status(self):
        age = (
            float("inf")
            if self.last_path_time is None
            else time.monotonic() - self.last_path_time
        )
        status = DiagnosticStatus()
        status.name = "aaa_edge_path_receiver"
        status.hardware_id = str(self.get_parameter("client_id").value)
        status.level = DiagnosticStatus.OK if self.connected and self.path_active else DiagnosticStatus.WARN
        status.message = "PATH_ACTIVE" if self.path_active else "STOPPED"
        status.values = [
            KeyValue(key="mqtt_connected", value=str(self.connected)),
            KeyValue(key="path_active", value=str(self.path_active)),
            KeyValue(key="path_age_sec", value=f"{age:.3f}"),
            KeyValue(key="last_sequence", value=str(self.last_sequence)),
        ]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.status_publisher.publish(output)

    def close(self):
        self._empty_path()
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    rclpy.init(args=args)
    node = MqttPathReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
