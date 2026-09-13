import copy
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .algorithms import euclidean_distance_transform


class EsdfMapper(Node):
    def __init__(self):
        super().__init__("esdf_mapper")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("output_topic", "/esdf_map")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("inflation_radius", 0.22)
        self.declare_parameter("max_distance", 1.5)
        self.declare_parameter("unknown_cost", 55)
        self.declare_parameter("padding_meters", 0.0)
        self.declare_parameter("map_subscription_transient_local", True)

        self.occupied_threshold = int(
            self.get_parameter("occupied_threshold").value
        )
        self.inflation_radius = float(
            self.get_parameter("inflation_radius").value
        )
        self.max_distance = max(
            0.01, float(self.get_parameter("max_distance").value)
        )
        self.unknown_cost = int(self.get_parameter("unknown_cost").value)
        self.padding_meters = max(
            0.0, float(self.get_parameter("padding_meters").value)
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if bool(
                    self.get_parameter(
                        "map_subscription_transient_local"
                    ).value
                )
                else DurabilityPolicy.VOLATILE
            ),
        )
        self.publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("output_topic").value),
            output_qos,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            input_qos,
        )

    def map_callback(self, message: OccupancyGrid):
        height = int(message.info.height)
        width = int(message.info.width)
        if not height or not width or len(message.data) != height * width:
            self.get_logger().warning("Ignoring malformed occupancy grid")
            return
        raw = np.asarray(message.data, dtype=np.int16).reshape(height, width)
        resolution = float(message.info.resolution)
        padding_cells = int(
            math.ceil(self.padding_meters / max(resolution, 1e-6))
        )
        output_info = copy.deepcopy(message.info)
        if padding_cells:
            raw = np.pad(
                raw,
                padding_cells,
                mode="constant",
                constant_values=-1,
            )
            height, width = raw.shape
            offset = padding_cells * resolution
            orientation = message.info.origin.orientation
            yaw = math.atan2(
                2.0
                * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0
                - 2.0
                * (
                    orientation.y * orientation.y
                    + orientation.z * orientation.z
                ),
            )
            output_info.origin.position.x -= offset * (
                math.cos(yaw) - math.sin(yaw)
            )
            output_info.origin.position.y -= offset * (
                math.sin(yaw) + math.cos(yaw)
            )
            output_info.width = width
            output_info.height = height
        obstacles = raw >= self.occupied_threshold
        # Never plan outside the represented map.
        obstacles[0, :] = True
        obstacles[-1, :] = True
        obstacles[:, 0] = True
        obstacles[:, -1] = True
        distance = (
            euclidean_distance_transform(obstacles)
            * resolution
        )
        clearance = np.maximum(distance - self.inflation_radius, 0.0)
        costs = np.rint(
            100.0
            * (1.0 - np.minimum(clearance, self.max_distance) / self.max_distance)
        ).astype(np.int16)
        costs[obstacles | (distance <= self.inflation_radius)] = 100
        unknown = raw < 0
        # Unknown space remains traversable for online exploration, but never
        # becomes cheaper than the configured unknown penalty.
        costs[unknown] = np.maximum(costs[unknown], self.unknown_cost)

        output = OccupancyGrid()
        output.header = message.header
        output.info = output_info
        output.data = np.clip(costs, 0, 100).astype(np.int8).ravel().tolist()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = EsdfMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
