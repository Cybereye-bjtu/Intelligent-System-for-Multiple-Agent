import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


ROBOT_STYLES = {
    "hyzx001": {
        "frame": "hyzx001/base_footprint",
        "length": 0.27,
        "width": 0.185,
        "color": (0.10, 0.55, 1.00),
    },
    "jetson003": {
        "frame": "jetson003/base_link",
        "length": 0.277,
        "width": 0.212,
        "color": (1.00, 0.42, 0.12),
    },
}


class RobotVisualizer(Node):
    """Publish lightweight physical-robot bodies attached to their TF frames."""

    def __init__(self):
        super().__init__("robot_visualizer")
        self.declare_parameter("safety_radius", 0.35)
        self.emergency_stop = True
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            MarkerArray, "/swarm/robot_markers", qos
        )
        self.create_subscription(
            Bool, "/swarm/emergency_stop", self._stop_callback, qos
        )
        self.create_timer(0.2, self._publish)

    def _stop_callback(self, message):
        self.emergency_stop = bool(message.data)

    @staticmethod
    def _marker(robot, frame, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = frame
        marker.ns = f"{robot}_vehicle"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _robot_markers(self, robot, style):
        frame = style["frame"]
        red, green, blue = style["color"]
        if self.emergency_stop:
            red, green, blue = 0.95, 0.12, 0.08

        body = self._marker(robot, frame, 0, Marker.CUBE)
        body.pose.position.z = 0.07
        body.scale.x = style["length"]
        body.scale.y = style["width"]
        body.scale.z = 0.12
        body.color.r = red
        body.color.g = green
        body.color.b = blue
        body.color.a = 0.92

        direction = self._marker(robot, frame, 1, Marker.ARROW)
        direction.pose.position.z = 0.15
        direction.scale.x = style["length"] * 0.9
        direction.scale.y = max(0.05, style["width"] * 0.28)
        direction.scale.z = direction.scale.y
        direction.color.r = red
        direction.color.g = green
        direction.color.b = blue
        direction.color.a = 1.0

        radius = float(self.get_parameter("safety_radius").value)
        ring = self._marker(robot, frame, 2, Marker.LINE_STRIP)
        ring.pose.position.z = 0.025
        ring.scale.x = 0.015
        for index in range(49):
            angle = 6.283185307179586 * index / 48.0
            ring.points.append(
                Point(
                    x=radius * math.cos(angle),
                    y=radius * math.sin(angle),
                )
            )
        ring.color.r = 1.0 if self.emergency_stop else red
        ring.color.g = 0.10 if self.emergency_stop else green
        ring.color.b = 0.10 if self.emergency_stop else blue
        ring.color.a = 0.65

        label = self._marker(robot, frame, 3, Marker.TEXT_VIEW_FACING)
        label.pose.position.z = 0.28
        label.scale.z = 0.12
        label.text = f"{robot}  {'ESTOP' if self.emergency_stop else 'READY'}"
        label.color.r = red
        label.color.g = green
        label.color.b = blue
        label.color.a = 1.0
        return body, direction, ring, label

    def _publish(self):
        output = MarkerArray()
        for robot, style in ROBOT_STYLES.items():
            markers = self._robot_markers(robot, style)
            output.markers.extend(markers)
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = RobotVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
