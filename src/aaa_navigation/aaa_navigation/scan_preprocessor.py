import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanPreprocessor(Node):
    def __init__(self):
        super().__init__("scan_preprocessor")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("filter_rear_self_returns", False)
        self.declare_parameter("rear_start_angle_deg", 125.0)
        self.declare_parameter("rear_self_return_max", 0.30)

        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.window = max(1, int(self.get_parameter("median_window").value))
        if self.window % 2 == 0:
            self.window += 1
        self.filter_rear = bool(
            self.get_parameter("filter_rear_self_returns").value
        )
        self.rear_angle = math.radians(
            float(self.get_parameter("rear_start_angle_deg").value)
        )
        self.rear_range = float(
            self.get_parameter("rear_self_return_max").value
        )
        self.publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("input_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )

    def scan_callback(self, message: LaserScan):
        values = np.asarray(message.ranges, dtype=np.float64)
        invalid = ~np.isfinite(values) | (values < self.range_min)
        values[invalid] = np.inf
        values[values > self.range_max] = np.inf

        if self.window > 1 and values.size >= self.window:
            radius = self.window // 2
            padded = np.pad(values, radius, mode="edge")
            values = np.median(
                np.lib.stride_tricks.sliding_window_view(
                    padded, self.window
                ),
                axis=1,
            )

        if self.filter_rear:
            angles = message.angle_min + np.arange(values.size) * message.angle_increment
            rear_self = (np.abs(angles) >= self.rear_angle) & (
                values <= self.rear_range
            )
            values[rear_self] = np.inf

        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = max(message.range_min, self.range_min)
        output.range_max = min(message.range_max, self.range_max)
        output.ranges = values.astype(np.float32).tolist()
        output.intensities = message.intensities
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
