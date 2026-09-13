from __future__ import annotations

import json
import math
import time
import uuid
from functools import partial
from typing import Dict

import rclpy
from aaa_search_interfaces.msg import NavigationStatus, NavigationTask, TargetObservation
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener

from .state_machine import MissionState, MissionStateMachine
from .approach_dispatch import order_by_assigned_goal_distance


class MissionManager(Node):
    """Owns task-level navigation-goal authority during search.

    V1 responsibilities:
      * IDLE -> EXPLORING when a non-empty natural-language query arrives.
      * Forward frontier/exploration goals to the existing per-robot goal_pose
        topics only while EXPLORING.
      * EXPLORING -> TARGET_FOUND on a validated target pose in site_map.
      * Immediately disable all further exploration-goal forwarding.

    Validated approach tasks are also forwarded to the existing per-robot
    ``goal_pose`` topics so the A* navigation chain can execute them.
    """

    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('robot_ids', ['hyzx001', 'jetson003'])
        self.declare_parameter('global_frame', 'site_map')
        self.declare_parameter('query_topic', '/search/mission_query')
        self.declare_parameter('target_found_topic', '/search/target_found')
        self.declare_parameter('global_observation_topic', '/search/target_observation_global')
        self.declare_parameter('mission_id_topic', '/search/mission_id')
        self.declare_parameter('accepted_query_topic', '/search/accepted_mission_query')
        self.declare_parameter('accept_legacy_target_found', False)
        self.declare_parameter('navigation_order', ['hyzx001', 'jetson003'])
        self.declare_parameter(
            'base_frames', ['hyzx001/base_footprint', 'jetson003/base_link']
        )
        self.declare_parameter('approach_goal_tolerance_m', 0.15)
        self.declare_parameter('approach_goal_dwell_sec', 0.5)
        self.declare_parameter('approach_task_collection_sec', 1.0)
        self.declare_parameter('second_approach_wait_timeout_sec', 30.0)
        self.declare_parameter('state_topic', '/search/state')
        self.declare_parameter('exploration_enabled_topic', '/search/exploration_enabled')
        self.declare_parameter('target_pose_topic', '/search/target_pose')
        self.declare_parameter('reset_topic', '/search/reset')

        self.robot_ids = list(self.get_parameter('robot_ids').value)
        self.global_frame = str(self.get_parameter('global_frame').value)
        query_topic = str(self.get_parameter('query_topic').value)
        target_found_topic = str(self.get_parameter('target_found_topic').value)
        global_observation_topic = str(
            self.get_parameter('global_observation_topic').value
        )
        state_topic = str(self.get_parameter('state_topic').value)
        exploration_enabled_topic = str(
            self.get_parameter('exploration_enabled_topic').value
        )
        target_pose_topic = str(self.get_parameter('target_pose_topic').value)
        reset_topic = str(self.get_parameter('reset_topic').value)

        self.sm = MissionStateMachine()
        self.mission_id = ''
        self.approach_tasks: Dict[str, NavigationTask] = {}
        self.current_goal_id = ''
        self.navigation_order = [
            str(value) for value in self.get_parameter('navigation_order').value
        ]
        if set(self.navigation_order) != set(self.robot_ids):
            raise ValueError('navigation_order must contain every configured robot exactly once')
        base_frames = [str(value) for value in self.get_parameter('base_frames').value]
        if len(base_frames) != len(self.robot_ids):
            raise ValueError('base_frames must contain one frame for every configured robot')
        self.base_frames = dict(zip(self.robot_ids, base_frames))
        self.approach_goal_tolerance_m = float(
            self.get_parameter('approach_goal_tolerance_m').value
        )
        self.approach_goal_dwell_sec = float(
            self.get_parameter('approach_goal_dwell_sec').value
        )
        self.approach_task_collection_sec = float(
            self.get_parameter('approach_task_collection_sec').value
        )
        self.second_approach_wait_timeout_sec = float(
            self.get_parameter('second_approach_wait_timeout_sec').value
        )
        self.navigation_sequence = list(self.navigation_order)
        self.approach_wait_started = None
        self.second_task_wait_started = None
        self.dispatch_monotonic = None
        self.goal_reached_since = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(String, state_topic, latched_qos)
        self.enabled_pub = self.create_publisher(Bool, exploration_enabled_topic, latched_qos)
        self.target_pose_pub = self.create_publisher(PoseStamped, target_pose_topic, latched_qos)
        self.mission_id_pub = self.create_publisher(
            String, str(self.get_parameter('mission_id_topic').value), latched_qos
        )
        self.accepted_query_pub = self.create_publisher(
            String, str(self.get_parameter('accepted_query_topic').value), latched_qos
        )

        self.create_subscription(String, query_topic, self._on_query, 10)
        self.create_subscription(
            TargetObservation,
            global_observation_topic,
            self._on_global_observation,
            10,
        )
        if bool(self.get_parameter('accept_legacy_target_found').value):
            self.create_subscription(PoseStamped, target_found_topic, self._on_target_found, 10)
        self.create_subscription(Empty, reset_topic, self._on_reset, 10)

        self.goal_pubs: Dict[str, object] = {}
        self.goal_subs = []
        self.navigation_task_pubs = {}
        self.cancel_pubs = {}
        self.approach_subs = []
        self.navigation_status_subs = []
        for robot_id in self.robot_ids:
            input_topic = f'/search/exploration_goal/{robot_id}'
            output_topic = f'/{robot_id}/goal_pose'
            self.goal_pubs[robot_id] = self.create_publisher(PoseStamped, output_topic, 10)
            self.goal_subs.append(
                self.create_subscription(
                    PoseStamped,
                    input_topic,
                    partial(self._on_exploration_goal, robot_id),
                    10,
                )
            )
            self.navigation_task_pubs[robot_id] = self.create_publisher(
                NavigationTask, f'/{robot_id}/navigation/task', 10
            )
            self.cancel_pubs[robot_id] = self.create_publisher(
                String, f'/{robot_id}/navigation/cancel', 10
            )
            self.approach_subs.append(
                self.create_subscription(
                    NavigationTask,
                    f'/search/approach_task/{robot_id}',
                    partial(self._on_approach_task, robot_id),
                    10,
                )
            )
            self.navigation_status_subs.append(
                self.create_subscription(
                    NavigationStatus,
                    f'/{robot_id}/navigation/status',
                    partial(self._on_navigation_status, robot_id),
                    10,
                )
            )

        self.navigation_monitor_timer = self.create_timer(
            0.1, self._monitor_navigation
        )

        self._publish_state()
        self.get_logger().info(
            f'Mission manager ready. robots={self.robot_ids} global_frame={self.global_frame}'
        )

    def _publish_state(self) -> None:
        state_msg = String()
        state_msg.data = self.sm.state.value
        self.state_pub.publish(state_msg)

        enabled_msg = Bool()
        enabled_msg.data = self.sm.state == MissionState.EXPLORING
        self.enabled_pub.publish(enabled_msg)

    def _on_query(self, msg: String) -> None:
        query = msg.data.strip()
        if self.sm.start(query):
            self.mission_id = uuid.uuid4().hex
            self.approach_tasks.clear()
            self.current_goal_id = ''
            self.navigation_sequence = list(self.navigation_order)
            self.approach_wait_started = None
            self.second_task_wait_started = None
            mission_message = String()
            mission_message.data = self.mission_id
            self.mission_id_pub.publish(mission_message)
            self.accepted_query_pub.publish(
                String(data=json.dumps({
                    'mission_id': self.mission_id,
                    'text': query,
                }, ensure_ascii=False, separators=(',', ':')))
            )
            self.get_logger().info(f'Mission started: {query!r} -> EXPLORING')
            self._publish_state()
            return

        self.get_logger().warning(
            f'Ignored mission query {query!r} while state={self.sm.state.value}'
        )

    def _valid_pose(self, msg: PoseStamped) -> bool:
        p = msg.pose.position
        values = (p.x, p.y, p.z)
        return all(math.isfinite(float(v)) for v in values)

    def _in_global_frame(self, msg: PoseStamped) -> bool:
        return msg.header.frame_id == self.global_frame

    def _on_exploration_goal(self, robot_id: str, msg: PoseStamped) -> None:
        if self.sm.state != MissionState.EXPLORING:
            self.get_logger().warning(
                f'Dropped exploration goal for {robot_id} because state={self.sm.state.value}'
            )
            return
        if not self._in_global_frame(msg):
            self.get_logger().error(
                f'Dropped exploration goal for {robot_id}: frame_id={msg.header.frame_id!r}, '
                f'expected {self.global_frame!r}'
            )
            return
        if not self._valid_pose(msg):
            self.get_logger().error(f'Dropped non-finite exploration goal for {robot_id}')
            return

        self.goal_pubs[robot_id].publish(msg)
        self.get_logger().info(
            f'Forwarded exploration goal: {robot_id} -> '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
        )

    def _on_target_found(self, msg: PoseStamped) -> None:
        if self.sm.state != MissionState.EXPLORING:
            self.get_logger().warning(
                f'Ignored target_found because state={self.sm.state.value}'
            )
            return
        if not self._in_global_frame(msg):
            self.get_logger().error(
                f'Rejected target_found: frame_id={msg.header.frame_id!r}, '
                f'expected {self.global_frame!r}'
            )
            return
        if not self._valid_pose(msg):
            self.get_logger().error('Rejected target_found with non-finite position')
            return

        if not self.sm.target_found():
            return

        # Cancel both planners before approach generation.  Each A* planner
        # clears its latched path, which makes VFH publish zero velocity.
        self._cancel_navigation_tasks()

        # Publish the confirmed fleet-global target pose for downstream consumers.
        self.target_pose_pub.publish(msg)
        self._publish_state()
        self.get_logger().warning(
            f'TARGET_FOUND at ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}) '
            f'in {self.global_frame}. Exploration goal forwarding disabled.'
        )

    def _on_global_observation(self, msg: TargetObservation) -> None:
        if self.sm.state != MissionState.EXPLORING:
            return
        if msg.mission_id != self.mission_id:
            self.get_logger().warning('Ignored target observation from a stale mission')
            return
        if msg.header.frame_id != self.global_frame:
            self.get_logger().error('Rejected global target observation outside site_map')
            return
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.position
        pose.pose.orientation.w = 1.0
        if self.sm.target_found():
            self._cancel_navigation_tasks()
            self.target_pose_pub.publish(pose)
            self._publish_state()
            self.get_logger().warning(
                f'TARGET_FOUND by {msg.robot_id}; waiting for reachable approach tasks'
            )

    def _on_approach_task(self, robot_id: str, msg: NavigationTask) -> None:
        if self.sm.state not in (
            MissionState.TARGET_FOUND,
            MissionState.NAVIGATING_ROBOT_1,
            MissionState.NAVIGATING_ROBOT_2,
        ):
            return
        if (
            msg.mission_id != self.mission_id
            or msg.robot_id != robot_id
            or msg.header.frame_id != self.global_frame
            or not msg.goal_id.strip()
        ):
            self.get_logger().warning(f'Ignored invalid/stale approach task for {robot_id}')
            return
        self.approach_tasks[robot_id] = msg
        if self.sm.state == MissionState.TARGET_FOUND:
            if len(self.approach_tasks) == len(self.robot_ids):
                self._start_approach_navigation()
            elif self.approach_wait_started is None:
                self.approach_wait_started = time.monotonic()
                self.get_logger().warning(
                    f'Only {robot_id} has a reachable approach task; '
                    'briefly waiting for a complete pair'
                )
        elif (
            self.sm.state == MissionState.NAVIGATING_ROBOT_2
            and not self.current_goal_id
            and robot_id == self.navigation_sequence[1]
        ):
            self.second_task_wait_started = None
            self._dispatch_navigation(robot_id)

    def _start_approach_navigation(self) -> None:
        if self.sm.state != MissionState.TARGET_FOUND or not self.approach_tasks:
            return
        assigned_goals = {
            robot_id: (task.goal.position.x, task.goal.position.y)
            for robot_id, task in self.approach_tasks.items()
        }
        robot_positions = {}
        try:
            for robot_id in assigned_goals:
                transform = self.tf_buffer.lookup_transform(
                    self.global_frame, self.base_frames[robot_id], Time()
                )
                robot_positions[robot_id] = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                )
            sequence, distances = order_by_assigned_goal_distance(
                self.navigation_order,
                assigned_goals,
                robot_positions,
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(
                f'Waiting to rank approach tasks by assigned-goal distance: {error}',
                throttle_duration_sec=2.0,
            )
            return
        self.navigation_sequence = sequence
        ranking = ', '.join(
            f'{robot_id}={distances[robot_id]:.3f}m'
            for robot_id in sequence if robot_id in distances
        )
        self.get_logger().info(f'Approach navigation order: {ranking}')
        self.approach_wait_started = None
        if self.sm.approach_ready():
            self._publish_state()
            self._dispatch_navigation(self.navigation_sequence[0])

    def _dispatch_navigation(self, robot_id: str) -> None:
        task = self.approach_tasks[robot_id]
        self.current_goal_id = task.goal_id
        self.navigation_task_pubs[robot_id].publish(task)
        goal = PoseStamped()
        goal.header = task.header
        goal.pose = task.goal
        self.goal_pubs[robot_id].publish(goal)
        self.dispatch_monotonic = time.monotonic()
        self.goal_reached_since = None
        self.get_logger().warning(
            f'Dispatched {task.goal_id!r} to {robot_id}; other robot remains stopped'
        )

    def _active_navigation_robot(self):
        if self.sm.state == MissionState.NAVIGATING_ROBOT_1:
            return self.navigation_sequence[0]
        if self.sm.state == MissionState.NAVIGATING_ROBOT_2:
            return self.navigation_sequence[1]
        return None

    def _monitor_navigation(self) -> None:
        now = time.monotonic()
        if (
            self.sm.state == MissionState.TARGET_FOUND
            and self.approach_tasks
            and self.approach_wait_started is not None
            and now - self.approach_wait_started >= self.approach_task_collection_sec
        ):
            self._start_approach_navigation()
            return
        robot_id = self._active_navigation_robot()
        if robot_id is None:
            self.goal_reached_since = None
            return
        if not self.current_goal_id:
            if (
                self.sm.state == MissionState.NAVIGATING_ROBOT_2
                and self.second_task_wait_started is not None
                and now - self.second_task_wait_started
                >= self.second_approach_wait_timeout_sec
            ):
                if self.sm.fail():
                    self._publish_state()
                    self.get_logger().error(
                        f'No reachable approach task for {robot_id} after '
                        f'{self.second_approach_wait_timeout_sec:.1f}s; safely stopped'
                    )
                self.second_task_wait_started = None
            self.goal_reached_since = None
            return
        task = self.approach_tasks.get(robot_id)
        if task is None:
            return
        if (
            self.dispatch_monotonic is not None
            and task.timeout_sec > 0.0
            and now - self.dispatch_monotonic > float(task.timeout_sec)
        ):
            self._emit_internal_navigation_status(
                robot_id, NavigationStatus.TIMEOUT, float('inf'), 'approach timeout'
            )
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frames[robot_id], Time()
            )
        except TransformException:
            self.goal_reached_since = None
            return
        dx = float(task.goal.position.x) - float(transform.transform.translation.x)
        dy = float(task.goal.position.y) - float(transform.transform.translation.y)
        distance = math.hypot(dx, dy)
        if distance > self.approach_goal_tolerance_m:
            self.goal_reached_since = None
            return
        if self.goal_reached_since is None:
            self.goal_reached_since = now
            return
        if now - self.goal_reached_since >= self.approach_goal_dwell_sec:
            self._emit_internal_navigation_status(
                robot_id, NavigationStatus.SUCCEEDED, distance, 'approach goal reached'
            )

    def _emit_internal_navigation_status(
        self, robot_id: str, status: int, distance: float, detail: str
    ) -> None:
        message = NavigationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.global_frame
        message.mission_id = self.mission_id
        message.goal_id = self.current_goal_id
        message.robot_id = robot_id
        message.status = status
        message.distance_remaining = float(distance)
        message.detail = detail
        self._on_navigation_status(robot_id, message)

    def _on_navigation_status(self, robot_id: str, msg: NavigationStatus) -> None:
        if (
            msg.mission_id != self.mission_id
            or msg.goal_id != self.current_goal_id
            or msg.robot_id != robot_id
        ):
            return
        expected_index = (
            0 if self.sm.state == MissionState.NAVIGATING_ROBOT_1
            else 1 if self.sm.state == MissionState.NAVIGATING_ROBOT_2
            else -1
        )
        if expected_index < 0 or robot_id != self.navigation_sequence[expected_index]:
            return
        if msg.status == NavigationStatus.SUCCEEDED:
            if expected_index == 0 and self.sm.first_robot_succeeded():
                self.current_goal_id = ''
                self.dispatch_monotonic = None
                self.goal_reached_since = None
                self._publish_state()
                next_robot = self.navigation_sequence[1]
                if next_robot in self.approach_tasks:
                    self._dispatch_navigation(next_robot)
                else:
                    self.second_task_wait_started = time.monotonic()
                    self.get_logger().warning(
                        f'{robot_id} reached its approach pose; waiting for a '
                        f'reachable approach task for {next_robot}'
                    )
            elif expected_index == 1 and self.sm.second_robot_succeeded():
                self.current_goal_id = ''
                self.dispatch_monotonic = None
                self.goal_reached_since = None
                self._publish_state()
                self.get_logger().warning('Both robots reached their target approach poses')
            return
        if msg.status in (
            NavigationStatus.PLAN_FAILED,
            NavigationStatus.BLOCKED,
            NavigationStatus.TIMEOUT,
            NavigationStatus.CANCELLED,
        ):
            if self.sm.fail():
                self._cancel_navigation_tasks()
                self._publish_state()
                self.get_logger().error(
                    f'Navigation failed for {robot_id}: {msg.detail}'
                )

    def _cancel_navigation_tasks(self) -> None:
        message = String()
        message.data = self.current_goal_id
        for publisher in self.cancel_pubs.values():
            publisher.publish(message)
        self.current_goal_id = ''
        self.dispatch_monotonic = None
        self.goal_reached_since = None

    def _on_reset(self, _msg: Empty) -> None:
        self._cancel_navigation_tasks()
        self.sm.reset()
        self.mission_id = ''
        self.approach_tasks.clear()
        self.navigation_sequence = list(self.navigation_order)
        self.approach_wait_started = None
        self.second_task_wait_started = None
        self._publish_state()
        self.get_logger().info('Mission reset -> IDLE')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
