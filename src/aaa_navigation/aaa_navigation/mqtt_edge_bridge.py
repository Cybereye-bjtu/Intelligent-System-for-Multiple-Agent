"""Bridge Jetson ROS telemetry and fail-safe commands over MQTT."""

import json
import math
import os
import ssl
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class MqttEdgeBridge(Node):
    """Vehicle-side bridge with a 200 ms command watchdog."""

    def __init__(self):
        super().__init__("jetson003_mqtt_edge_bridge")
        defaults = {
            "broker": "",
            "port": 8883,
            "keepalive_seconds": 60,
            "client_id": "aaa-jetson003-edge",
            "username_env": "HYZX_MQTT_USERNAME",
            "password_env": "HYZX_MQTT_PASSWORD",
            "ca_file": "/etc/ssl/certs/ca-certificates.crt",
            "tls_insecure": False,
            "source": "jetson003",
            "scan_ros_topic": "/scan",
            "odom_ros_topic": "/odom",
            "tf_ros_topic": "/tf",
            "tf_static_ros_topic": "/tf_static",
            "command_ros_topic": "/cmd_vel_edge_nominal",
            "scan_mqtt_topic": "robot/jetson003/telemetry/scan",
            "odom_mqtt_topic": "robot/jetson003/telemetry/odom",
            "tf_mqtt_topic": "robot/jetson003/telemetry/tf",
            "tf_static_mqtt_topic": "robot/jetson003/telemetry/tf_static",
            "status_mqtt_topic": "robot/jetson003/telemetry/status",
            "command_mqtt_topic": "edge/jetson003/car/cmd_vel",
            "command_timeout": 0.20,
            "command_frequency": 20.0,
            "status_frequency": 1.0,
            "max_linear_velocity": 0.10,
            "max_angular_velocity": 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.source = str(self.get_parameter("source").value)
        self.sequence = {}
        self.connected = False
        self.shutting_down = False
        self.latest_command = Twist()
        self.last_command_time = None
        self.last_static_transforms = []
        self.command_lock = threading.Lock()

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_ros_topic").value), 1
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_ros_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_ros_topic").value),
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("tf_ros_topic").value),
            self.tf_callback,
            qos_profile_sensor_data,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("tf_static_ros_topic").value),
            self.tf_static_callback,
            static_qos,
        )

        self.client = self._create_client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self._configure_security()
        self.client.connect_async(
            str(self.get_parameter("broker").value),
            int(self.get_parameter("port").value),
            int(self.get_parameter("keepalive_seconds").value),
        )
        self.client.loop_start()

        command_frequency = max(
            1.0, float(self.get_parameter("command_frequency").value)
        )
        status_frequency = max(
            0.2, float(self.get_parameter("status_frequency").value)
        )
        self.create_timer(1.0 / command_frequency, self.publish_command)
        self.create_timer(1.0 / status_frequency, self.publish_status)

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

    def _configure_security(self):
        username = os.environ.get(
            str(self.get_parameter("username_env").value), ""
        )
        password = os.environ.get(
            str(self.get_parameter("password_env").value), ""
        )
        if not username or not password:
            raise RuntimeError(
                "MQTT credential environment variables are missing"
            )
        self.client.username_pw_set(username, password)
        self.client.tls_set(
            ca_certs=str(self.get_parameter("ca_file").value),
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
        self.client.tls_insecure_set(
            bool(self.get_parameter("tls_insecure").value)
        )

    @staticmethod
    def _stamp_ms(stamp):
        if stamp.sec > 0:
            return stamp.sec * 1000 + stamp.nanosec // 1_000_000
        return int(time.time() * 1000)

    def _publish(self, topic_parameter, data, qos=0, retain=False):
        if not self.connected:
            return
        topic = str(self.get_parameter(topic_parameter).value)
        sequence = self.sequence.get(topic, 0) + 1
        self.sequence[topic] = sequence
        payload = {
            "source": self.source,
            "seq": sequence,
            "ts": int(time.time() * 1000),
            "data": data,
        }
        self.client.publish(
            topic,
            json.dumps(payload, separators=(",", ":"), allow_nan=False),
            qos=qos,
            retain=retain,
        )

    def scan_callback(self, message):
        ranges = []
        for value in message.ranges:
            ranges.append(
                int(round(value * 1000.0))
                if math.isfinite(value) and value >= 0.0
                else 0
            )
        self._publish("scan_mqtt_topic", {
            "t": self._stamp_ms(message.header.stamp),
            "f": message.header.frame_id,
            "amin": message.angle_min,
            "amax": message.angle_max,
            "ainc": message.angle_increment,
            "rmin": message.range_min,
            "rmax": message.range_max,
            "r": ranges,
        })

    def odom_callback(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        self._publish("odom_mqtt_topic", {
            "t": self._stamp_ms(message.header.stamp),
            "f": message.header.frame_id,
            "c": message.child_frame_id,
            "p": [pose.position.x, pose.position.y, pose.position.z],
            "o": [
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ],
            "v": [twist.linear.x, twist.linear.y, twist.linear.z],
            "w": [twist.angular.x, twist.angular.y, twist.angular.z],
        })

    def _transforms(self, message):
        output = []
        for transform in message.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            output.append({
                "t": self._stamp_ms(transform.header.stamp),
                "f": transform.header.frame_id,
                "c": transform.child_frame_id,
                "tr": [translation.x, translation.y, translation.z],
                "r": [rotation.x, rotation.y, rotation.z, rotation.w],
            })
        return output

    def tf_callback(self, message):
        self._publish("tf_mqtt_topic", self._transforms(message))

    def tf_static_callback(self, message):
        self.last_static_transforms = self._transforms(message)
        self._publish(
            "tf_static_mqtt_topic",
            self.last_static_transforms,
            qos=1,
            retain=True,
        )

    def on_connect(self, client, userdata, flags, result_code):
        self.connected = int(result_code) == 0
        if self.connected:
            client.subscribe(
                str(self.get_parameter("command_mqtt_topic").value), qos=1
            )
            self.get_logger().info(
                "MQTT connected; telemetry publishing active"
            )
            if self.last_static_transforms:
                self._publish(
                    "tf_static_mqtt_topic",
                    self.last_static_transforms,
                    qos=1,
                    retain=True,
                )
        else:
            self.get_logger().error(f"MQTT rejected connection: {result_code}")

    def on_disconnect(self, client, userdata, result_code):
        self.connected = False
        with self.command_lock:
            self.latest_command = Twist()
            self.last_command_time = None
        self.command_publisher.publish(Twist())
        if not self.shutting_down:
            self.get_logger().error("MQTT disconnected; command output zeroed")

    def on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            linear = float(payload.get("linear_x", 0.0))
            angular = float(payload.get("angular_z", 0.0))
            if not math.isfinite(linear) or not math.isfinite(angular):
                raise ValueError("non-finite velocity")
        except (
            UnicodeDecodeError,
            json.JSONDecodeError,
            TypeError,
            ValueError,
        ):
            return
        command = Twist()
        linear_limit = float(self.get_parameter("max_linear_velocity").value)
        angular_limit = float(self.get_parameter("max_angular_velocity").value)
        command.linear.x = max(-linear_limit, min(linear, linear_limit))
        command.angular.z = max(-angular_limit, min(angular, angular_limit))
        with self.command_lock:
            self.latest_command = command
            self.last_command_time = time.monotonic()

    def publish_command(self):
        output = Twist()
        with self.command_lock:
            fresh = self.last_command_time is not None
            if fresh:
                age = time.monotonic() - self.last_command_time
                fresh = age <= float(
                    self.get_parameter("command_timeout").value
                )
            if self.connected and fresh:
                output = self.latest_command
        self.command_publisher.publish(output)

    def publish_status(self):
        fresh = self.last_command_time is not None
        if fresh:
            age = time.monotonic() - self.last_command_time
            fresh = age <= float(
                self.get_parameter("command_timeout").value
            )
        self._publish("status_mqtt_topic", {
            "t": int(time.time() * 1000),
            "mqtt_connected": self.connected,
            "command_fresh": fresh,
        }, qos=1)

    def close(self):
        self.shutting_down = True
        self.command_publisher.publish(Twist())
        self.client.disconnect()
        self.client.loop_stop()


def main(args=None):
    rclpy.init(args=args)
    node = MqttEdgeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
