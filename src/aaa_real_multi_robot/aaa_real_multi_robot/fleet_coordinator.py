import math
import uuid

import numpy as np
import rclpy
from aaa_real_multi_robot_interfaces.msg import NavigationLease
from nav_msgs.msg import Path
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, UInt64
from tf2_ros import Buffer, TransformException, TransformListener


class FleetCoordinator(Node):
    """Fail closed and issue short motion leases after trajectory conflict checks."""

    def __init__(self):
        super().__init__("fleet_coordinator")
        self.declare_parameter("robot_names", ["hyzx001", "jetson003"])
        self.declare_parameter("base_frames", ["hyzx001/base_footprint", "jetson003/base_link"])
        self.declare_parameter("conflict_priority", ["hyzx001", "jetson003"])
        self.declare_parameter("global_frame", "site_map")
        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("lease_duration", 0.35)
        self.declare_parameter("nominal_speed", 0.10)
        self.declare_parameter("max_angular_speed", 0.20)
        self.declare_parameter("trajectory_resolution", 0.10)
        self.declare_parameter("safety_radius", 0.30)
        self.declare_parameter("time_window", 1.5)
        self.declare_parameter("coordination_horizon", 15.0)
        self.declare_parameter("winner_hold_time", 2.0)
        self.declare_parameter("path_timeout", 2.0)
        self.declare_parameter("tf_timeout", 0.50)
        self.robots = [str(name).strip("/") for name in self.get_parameter("robot_names").value]
        priority = [
            str(name).strip("/")
            for name in self.get_parameter("conflict_priority").value
        ]
        if sorted(priority) != sorted(self.robots):
            raise ValueError(
                "conflict_priority must contain every robot exactly once"
            )
        self.priority_rank = {name: index for index, name in enumerate(priority)}
        frames = [str(frame).lstrip("/") for frame in self.get_parameter("base_frames").value]
        if len(frames) != len(self.robots):
            raise ValueError("base_frames must contain one frame for each robot")
        self.base_frames = dict(zip(self.robots, frames))
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.paths = {}
        self.path_epoch = {}
        self.path_receipts = {}
        self.map_epoch = 0
        self.emergency_stop = True
        self.wait_started = {name: None for name in self.robots}
        self.pair_winners = {}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.lease_publishers = {
            name: self.create_publisher(NavigationLease, f"/{name}/fleet/lease", 5)
            for name in self.robots
        }
        self._path_subscriptions = [
            self.create_subscription(Path, f"/{name}/global_path", lambda message, robot=name: self._path_callback(robot, message), 5)
            for name in self.robots
        ]
        self.create_subscription(UInt64, "/swarm/map_epoch", self._epoch_callback, 5)
        emergency_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/swarm/emergency_stop", self._emergency_callback, emergency_qos)
        frequency = max(2.0, float(self.get_parameter("control_frequency").value))
        self.create_timer(1.0 / frequency, self._tick, clock=self.steady_clock)

    def _path_callback(self, robot, message):
        if message.header.frame_id != self.global_frame:
            self.get_logger().error(f"Rejected {robot} path in '{message.header.frame_id}', expected '{self.global_frame}'", throttle_duration_sec=2.0)
            return
        self.paths[robot] = message
        self.path_epoch[robot] = self.map_epoch
        self.path_receipts[robot] = self.steady_clock.now()

    def _epoch_callback(self, message):
        epoch = int(message.data)
        if epoch != self.map_epoch:
            self.map_epoch = epoch
            self.paths.clear()
            self.path_receipts.clear()
            self.pair_winners.clear()

    def _emergency_callback(self, message):
        self.emergency_stop = bool(message.data)

    def _pose(self, robot):
        transform = self.tf_buffer.lookup_transform(self.global_frame, self.base_frames[robot], Time(), timeout=Duration(seconds=0.03))
        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds <= 0:
            raise TransformException(f"{robot} transform has no timestamp")
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age < -0.10 or age > float(self.get_parameter("tf_timeout").value):
            raise TransformException(f"{robot} transform is stale by {age:.3f} seconds")
        return np.asarray([transform.transform.translation.x, transform.transform.translation.y], dtype=np.float64)

    def _trajectory(self, robot, pose):
        path = self.paths.get(robot)
        receipt = self.path_receipts.get(robot)
        fresh = receipt is not None and (
            self.steady_clock.now() - receipt
        ) <= Duration(seconds=float(self.get_parameter("path_timeout").value))
        if path is None or not fresh or self.path_epoch.get(robot) != self.map_epoch or len(path.poses) < 2:
            return None
        points = np.asarray([[pose_item.pose.position.x, pose_item.pose.position.y] for pose_item in path.poses], dtype=np.float64)
        start = int(np.argmin(np.linalg.norm(points - pose[None, :], axis=1)))
        points = np.vstack((pose, points[start:]))
        resolution = max(0.05, float(self.get_parameter("trajectory_resolution").value))
        dense = [points[0]]
        for first, second in zip(points[:-1], points[1:]):
            length = float(np.linalg.norm(second - first))
            count = max(1, int(math.ceil(length / resolution)))
            dense.extend(first + (second - first) * (index / count) for index in range(1, count + 1))
        dense = np.asarray(dense, dtype=np.float64)
        distances = np.zeros(len(dense), dtype=np.float64)
        if len(dense) > 1:
            distances[1:] = np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))
        times = distances / max(0.03, float(self.get_parameter("nominal_speed").value))
        keep = times <= float(self.get_parameter("coordination_horizon").value)
        return dense[keep], times[keep]

    def _conflict(self, first, second):
        points_a, times_a = first
        points_b, times_b = second
        distances = np.linalg.norm(points_a[:, None, :] - points_b[None, :, :], axis=2)
        time_delta = np.abs(times_a[:, None] - times_b[None, :])
        candidates = np.argwhere((distances < float(self.get_parameter("safety_radius").value)) & (time_delta < float(self.get_parameter("time_window").value)))
        if not len(candidates):
            return None
        index_a, index_b = min(candidates, key=lambda pair: max(times_a[int(pair[0])], times_b[int(pair[1])]))
        return float(times_a[index_a]), float(times_b[index_b])

    def _winner(self, first, second, arrival_times, now_seconds):
        key = tuple(sorted((first, second)))
        cached = self.pair_winners.get(key)
        if cached and cached[1] > now_seconds:
            return cached[0]
        del arrival_times
        winner = min((first, second), key=self.priority_rank.__getitem__)
        self.pair_winners[key] = (winner, now_seconds + float(self.get_parameter("winner_hold_time").value))
        return winner

    def _tick(self):
        now = self.get_clock().now()
        now_seconds = now.nanoseconds / 1e9
        permitted = {name: not self.emergency_stop for name in self.robots}
        trajectories = {}
        poses = {}
        for robot in self.robots:
            try:
                pose = self._pose(robot)
            except TransformException:
                permitted[robot] = False
                continue
            poses[robot] = pose
            trajectory = self._trajectory(robot, pose)
            if trajectory is not None:
                trajectories[robot] = trajectory
            else:
                permitted[robot] = False
        names = list(trajectories)
        for index, first in enumerate(names):
            for second in names[index + 1:]:
                conflict = self._conflict(trajectories[first], trajectories[second])
                if conflict is not None:
                    winner = self._winner(first, second, conflict, now_seconds)
                    permitted[second if winner == first else first] = False
        stationary = {
            robot: pose
            for robot, pose in poses.items()
            if robot not in trajectories
        }
        safety_radius = float(self.get_parameter("safety_radius").value)
        for moving_robot, (points, _) in trajectories.items():
            for stopped_robot, stopped_pose in stationary.items():
                if stopped_robot == moving_robot:
                    continue
                if np.any(np.linalg.norm(points - stopped_pose[None, :], axis=1) < safety_radius):
                    permitted[moving_robot] = False
        for robot in self.robots:
            if permitted[robot]:
                self.wait_started[robot] = None
            elif self.wait_started[robot] is None:
                self.wait_started[robot] = now_seconds
            self._publish_lease(robot, permitted[robot], now)

    def _publish_lease(self, robot, permit, now):
        message = NavigationLease()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.global_frame
        message.robot_id = self.robots.index(robot)
        message.permit_motion = bool(permit)
        message.max_linear_speed = float(self.get_parameter("nominal_speed").value)
        message.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        message.valid_until = (now + Duration(seconds=float(self.get_parameter("lease_duration").value))).to_msg()
        message.map_epoch = self.map_epoch
        message.reservation_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{robot}:{self.map_epoch}"))
        message.reason = "clear" if permit else "emergency stop, conflict, or missing TF"
        self.lease_publishers[robot].publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
