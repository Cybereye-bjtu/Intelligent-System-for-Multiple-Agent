import math

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .se2 import apply


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class MapFuser(Node):
    """Fuse the latest valid physical-robot maps in site_map."""

    def __init__(self):
        super().__init__("map_fuser")
        self.declare_parameter("global_frame", "site_map")
        self.declare_parameter("robot_names", ["hyzx001", "jetson003"])
        self.declare_parameter("map_topic_suffix", "map")
        self.declare_parameter("output_topic", "/swarm/map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("padding", 1.0)
        self.declare_parameter("max_output_cells", 8000000)
        self.declare_parameter("map_timeout", 5.0)
        self.declare_parameter("tf_timeout", 1.0)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robots = [str(name).strip("/") for name in self.get_parameter("robot_names").value]
        self.maps = {}
        self.map_receipts = {}
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, str(self.get_parameter("output_topic").value), qos)
        self.status_publisher = self.create_publisher(DiagnosticArray, "/swarm/map_fusion_status", qos)
        suffix = str(self.get_parameter("map_topic_suffix").value).strip("/")
        self._map_subscriptions = [
            self.create_subscription(OccupancyGrid, f"/{robot}/{suffix}", lambda message, name=robot: self._map_callback(name, message), qos)
            for robot in self.robots
        ]
        frequency = max(0.1, float(self.get_parameter("publish_frequency").value))
        self.create_timer(1.0 / frequency, self._fuse)

    def _map_callback(self, robot, message):
        expected = int(message.info.width) * int(message.info.height)
        if expected > 0 and len(message.data) == expected and message.header.frame_id:
            self.maps[robot] = message
            self.map_receipts[robot] = self.steady_clock.now()

    def _lookup(self, frame):
        transform = self.tf_buffer.lookup_transform(self.global_frame, frame, Time(), timeout=Duration(seconds=0.05))
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if stamp.nanoseconds <= 0 or age < -0.10 or age > float(self.get_parameter("tf_timeout").value):
            raise TransformException(f"transform for {frame} is stale by {age:.3f} seconds")
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            quaternion_yaw(transform.transform.rotation),
        )

    def _corners(self, message, transform):
        origin = message.info.origin
        local = (origin.position.x, origin.position.y, quaternion_yaw(origin.orientation))
        width = float(message.info.width) * float(message.info.resolution)
        height = float(message.info.height) * float(message.info.resolution)
        result = []
        for x, y in ((0.0, 0.0), (width, 0.0), (0.0, height), (width, height)):
            local_xy = apply(local, x, y)
            result.append(apply(transform, *local_xy))
        return result

    def _fuse(self):
        usable = []
        unaligned = []
        all_corners = []
        for robot, message in self.maps.items():
            receipt = self.map_receipts.get(robot)
            if receipt is None or (
                self.steady_clock.now() - receipt
            ) > Duration(seconds=float(self.get_parameter("map_timeout").value)):
                unaligned.append(robot)
                continue
            try:
                transform = self._lookup(message.header.frame_id)
            except TransformException:
                unaligned.append(robot)
                continue
            usable.append((robot, message, transform))
            all_corners.extend(self._corners(message, transform))
        if not usable:
            self._status([], unaligned or self.robots, "no aligned maps")
            return
        resolution = float(self.get_parameter("resolution").value)
        padding = float(self.get_parameter("padding").value)
        min_x = math.floor((min(point[0] for point in all_corners) - padding) / resolution) * resolution
        min_y = math.floor((min(point[1] for point in all_corners) - padding) / resolution) * resolution
        max_x = math.ceil((max(point[0] for point in all_corners) + padding) / resolution) * resolution
        max_y = math.ceil((max(point[1] for point in all_corners) + padding) / resolution) * resolution
        width = max(1, int(round((max_x - min_x) / resolution)))
        height = max(1, int(round((max_y - min_y) / resolution)))
        if width * height > int(self.get_parameter("max_output_cells").value):
            self._status([item[0] for item in usable], unaligned, "output grid exceeds safety limit", DiagnosticStatus.ERROR)
            return
        fused = np.full(width * height, -1, dtype=np.int16)
        occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        for _, message, transform in usable:
            source = np.asarray(message.data, dtype=np.int16).reshape((message.info.height, message.info.width))
            rows, columns = np.nonzero(source >= 0)
            if not rows.size:
                continue
            cell = float(message.info.resolution)
            x = (columns + 0.5) * cell
            y = (rows + 0.5) * cell
            origin = message.info.origin
            origin_yaw = quaternion_yaw(origin.orientation)
            cosine = math.cos(origin_yaw)
            sine = math.sin(origin_yaw)
            map_x = origin.position.x + cosine * x - sine * y
            map_y = origin.position.y + sine * x + cosine * y
            tx, ty, yaw = transform
            cosine = math.cos(yaw)
            sine = math.sin(yaw)
            global_x = tx + cosine * map_x - sine * map_y
            global_y = ty + sine * map_x + cosine * map_y
            output_columns = np.floor((global_x - min_x) / resolution).astype(np.int64)
            output_rows = np.floor((global_y - min_y) / resolution).astype(np.int64)
            valid = (output_columns >= 0) & (output_columns < width) & (output_rows >= 0) & (output_rows < height)
            indices = output_rows[valid] * width + output_columns[valid]
            values = source[rows[valid], columns[valid]].copy()
            values[values >= occupied_threshold] = 100
            np.maximum.at(fused, indices, values)
        output = OccupancyGrid()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.global_frame
        output.info.resolution = resolution
        output.info.width = width
        output.info.height = height
        output.info.origin.position.x = min_x
        output.info.origin.position.y = min_y
        output.info.origin.orientation.w = 1.0
        output.data = np.clip(fused, -1, 100).astype(np.int8).tolist()
        self.publisher.publish(output)
        aligned = [item[0] for item in usable]
        missing = [robot for robot in self.robots if robot not in aligned]
        self._status(aligned, sorted(set(unaligned + missing)), f"{len(aligned)}/{len(self.robots)} maps fused")

    def _status(self, aligned, unaligned, detail, level=DiagnosticStatus.OK):
        item = DiagnosticStatus()
        item.name = "swarm/map_fusion"
        item.hardware_id = "fleet"
        item.level = level if not unaligned else max(level, DiagnosticStatus.WARN)
        item.message = detail
        item.values = [KeyValue(key="aligned", value=",".join(aligned)), KeyValue(key="unaligned", value=",".join(unaligned))]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [item]
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MapFuser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
