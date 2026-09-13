"""Centralized two-robot WFD preview and optional exploration-goal dispatcher."""

from __future__ import annotations

import json
import math
from functools import partial
from typing import Dict, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .cooperative_wfd_core import (
    GridGeometry,
    GoalProgressTracker,
    align_grid_to_geometry,
    assign_frontiers,
    cell_to_world,
    detect_frontiers,
    path_costs,
    world_to_cell,
)


# Reuse the deployed map/A* safety conventions instead of introducing another
# set of tunable thresholds for exploration.
OCCUPIED_THRESHOLD = 65
LETHAL_COST = 100


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class CooperativeWfd(Node):
    """Extract shared-map frontiers and assign distinct previews to two robots."""

    def __init__(self) -> None:
        super().__init__('cooperative_wfd')
        self.declare_parameter('robot_ids', ['hyzx001', 'jetson003'])
        self.declare_parameter(
            'base_frames', ['hyzx001/base_footprint', 'jetson003/base_link']
        )
        self.declare_parameter(
            'esdf_topics', ['/hyzx001/esdf_map', '/jetson003/esdf_map']
        )
        self.declare_parameter('global_frame', 'site_map')
        self.declare_parameter('map_topic', '/swarm/map')
        self.declare_parameter('exploration_enabled_topic', '/search/exploration_enabled')
        self.declare_parameter('preview_topic_prefix', '/search/wfd/preview_goal')
        self.declare_parameter('goal_topic_prefix', '/search/exploration_goal')
        self.declare_parameter('marker_topic', '/search/wfd/markers')
        self.declare_parameter('status_topic', '/search/wfd/status')
        self.declare_parameter('min_frontier_length_m', 0.50)
        self.declare_parameter('frontier_standoff_distance_m', 0.30)
        self.declare_parameter('goal_tolerance_m', 0.25)
        self.declare_parameter('progress_timeout_s', 20.0)
        self.declare_parameter('planning_period_s', 2.0)
        self.declare_parameter('publish_navigation_goals', False)
        self.declare_parameter('tf_timeout_s', 0.50)

        robot_ids = [str(value).strip('/') for value in self.get_parameter('robot_ids').value]
        base_frames = [str(value).lstrip('/') for value in self.get_parameter('base_frames').value]
        esdf_topics = [str(value) for value in self.get_parameter('esdf_topics').value]
        if len(robot_ids) != 2 or len(base_frames) != 2 or len(esdf_topics) != 2:
            raise ValueError('cooperative_wfd requires exactly two robots, base frames, and ESDF topics')
        if len(set(robot_ids)) != 2:
            raise ValueError('robot_ids must be unique')
        self.robot_ids = robot_ids
        self.base_frames = dict(zip(robot_ids, base_frames))
        self.esdf_topics = dict(zip(robot_ids, esdf_topics))
        self.global_frame = str(self.get_parameter('global_frame').value)
        self.map_message = None
        self.esdf_messages: Dict[str, OccupancyGrid] = {}
        self.exploration_enabled = False
        self.last_summary = None
        self.dispatch_enabled = bool(
            self.get_parameter('publish_navigation_goals').value
        )
        self.goal_tolerance_m = float(
            self.get_parameter('goal_tolerance_m').value
        )
        self.progress = GoalProgressTracker(
            self.goal_tolerance_m,
            float(self.get_parameter('progress_timeout_s').value),
        )
        self.blacklisted_goals = {robot_id: [] for robot_id in self.robot_ids}
        self.pending_failures = set()
        self.last_events: Dict[str, str] = {}

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            latched,
        )
        self.esdf_subs = [
            self.create_subscription(
                OccupancyGrid,
                self.esdf_topics[robot_id],
                partial(self._on_esdf, robot_id),
                latched,
            )
            for robot_id in self.robot_ids
        ]
        self.enabled_sub = self.create_subscription(
            Bool,
            str(self.get_parameter('exploration_enabled_topic').value),
            self._on_enabled,
            latched,
        )
        self.path_subs = [
            self.create_subscription(
                Path,
                f'/{robot_id}/global_path',
                partial(self._on_path, robot_id),
                latched,
            )
            for robot_id in self.robot_ids
        ]
        preview_prefix = str(self.get_parameter('preview_topic_prefix').value).rstrip('/')
        goal_prefix = str(self.get_parameter('goal_topic_prefix').value).rstrip('/')
        self.preview_pubs = {
            robot_id: self.create_publisher(PoseStamped, f'{preview_prefix}/{robot_id}', 10)
            for robot_id in self.robot_ids
        }
        self.goal_pubs = {
            robot_id: self.create_publisher(PoseStamped, f'{goal_prefix}/{robot_id}', 10)
            for robot_id in self.robot_ids
        }
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), latched
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), latched
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        period = max(0.25, float(self.get_parameter('planning_period_s').value))
        self.timer = self.create_timer(period, self._plan)
        mode = 'armed dispatch' if self.dispatch_enabled else 'preview only'
        self.get_logger().info(f'Cooperative WFD ready ({mode}); map={self.get_parameter("map_topic").value}')

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _on_esdf(self, robot_id: str, message: OccupancyGrid) -> None:
        self.esdf_messages[robot_id] = message

    def _on_enabled(self, message: Bool) -> None:
        enabled = bool(message.data)
        if self.exploration_enabled and not enabled:
            self.progress.clear()
            self.pending_failures.clear()
            self.blacklisted_goals = {
                robot_id: [] for robot_id in self.robot_ids
            }
        self.exploration_enabled = enabled

    def _on_path(self, robot_id: str, message: Path) -> None:
        if (
            self.dispatch_enabled
            and self.exploration_enabled
            and self.progress.has_goal(robot_id)
            and len(message.poses) < 2
        ):
            self.pending_failures.add(robot_id)

    def _feedback_events(
        self,
        positions: Dict[str, Tuple[float, float]],
        now_s: float,
    ) -> Dict[str, str]:
        previous_goals = {
            robot_id: self.progress.goal(robot_id) for robot_id in self.robot_ids
        }
        events = self.progress.update(positions, now_s)
        for robot_id in self.pending_failures:
            if previous_goals.get(robot_id) is not None:
                events[robot_id] = 'path_failed'
                self.progress.clear(robot_id)
        self.pending_failures.clear()
        timeout = float(self.get_parameter('progress_timeout_s').value)
        for robot_id, event in events.items():
            goal_xy = previous_goals.get(robot_id)
            if goal_xy is not None:
                self.blacklisted_goals[robot_id].append(
                    (goal_xy, now_s + 2.0 * timeout)
                )
            self.get_logger().warning(
                f'WFD goal {event} for {robot_id}; requesting reassignment'
            )
        self.last_events = events
        return events

    def _apply_goal_exclusions(
        self,
        distances: Dict[str, np.ndarray],
        frontiers,
        geometry: GridGeometry,
        now_s: float,
    ) -> None:
        radius = max(0.50, 2.0 * self.goal_tolerance_m)
        exclusions = []
        for robot_id in self.robot_ids:
            retained = [
                item for item in self.blacklisted_goals[robot_id]
                if item[1] > now_s
            ]
            self.blacklisted_goals[robot_id] = retained
            exclusions.extend(goal for goal, _expiry in retained)
            active = self.progress.goal(robot_id)
            if active is not None:
                # Active frontiers are fleet reservations, not candidates for
                # the other robot until arrival/stall/path failure.
                exclusions.append(active)
        for frontier in frontiers:
            near_exclusion = any(
                math.dist(cell_to_world(cell, geometry), point) <= radius
                for point in exclusions
                for cell in frontier.cells
            )
            if near_exclusion:
                for values in distances.values():
                    for cell in frontier.cells:
                        values[cell] = np.inf

    @staticmethod
    def _grid_array(message: OccupancyGrid) -> np.ndarray:
        height = int(message.info.height)
        width = int(message.info.width)
        if height <= 0 or width <= 0 or len(message.data) != height * width:
            raise ValueError('malformed OccupancyGrid')
        return np.asarray(message.data, dtype=np.int16).reshape(height, width)

    @staticmethod
    def _geometry(message: OccupancyGrid) -> GridGeometry:
        info = message.info
        return GridGeometry(
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=quaternion_yaw(info.origin.orientation),
        )

    def _robot_positions(self) -> Dict[str, Tuple[float, float]]:
        positions = {}
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_s').value))
        for robot_id in self.robot_ids:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frames[robot_id], Time(), timeout=timeout
            )
            positions[robot_id] = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
        return positions

    def _publish_status(self, document: dict) -> None:
        self.status_pub.publish(String(data=json.dumps(document, separators=(',', ':'))))
        summary = (document.get('state'), document.get('frontier_count'), document.get('assignments'))
        if summary != self.last_summary:
            self.last_summary = summary
            self.get_logger().info(json.dumps(document, sort_keys=True))

    def _pose(self, robot_xy, goal_xy, stamp) -> PoseStamped:
        yaw = math.atan2(goal_xy[1] - robot_xy[1], goal_xy[0] - robot_xy[0])
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.global_frame
        message.pose.position.x = float(goal_xy[0])
        message.pose.position.y = float(goal_xy[1])
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def _publish_markers(self, frontiers, assignments, positions, geometry, stamp) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)

        points = Marker()
        points.header.frame_id = self.global_frame
        points.header.stamp = stamp
        points.ns = 'wfd_frontiers'
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = max(0.035, geometry.resolution * 0.8)
        points.scale.y = points.scale.x
        points.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=0.85)
        for frontier in frontiers:
            for cell in frontier.cells:
                x, y = cell_to_world(cell, geometry)
                points.points.append(Point(x=x, y=y, z=0.05))
        output.markers.append(points)

        colors = (
            ColorRGBA(r=0.15, g=1.0, b=0.25, a=1.0),
            ColorRGBA(r=1.0, g=0.35, b=0.05, a=1.0),
        )
        marker_id = 1
        for robot_index, robot_id in enumerate(self.robot_ids):
            assignment = assignments.get(robot_id)
            if assignment is None:
                continue
            goal_x, goal_y = cell_to_world(assignment.goal_cell, geometry)
            sphere = Marker()
            sphere.header.frame_id = self.global_frame
            sphere.header.stamp = stamp
            sphere.ns = 'wfd_assignments'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = goal_x
            sphere.pose.position.y = goal_y
            sphere.pose.position.z = 0.10
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.18
            sphere.color = colors[robot_index]
            output.markers.append(sphere)

            line = Marker()
            line.header = sphere.header
            line.ns = 'wfd_assignment_links'
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.035
            line.color = colors[robot_index]
            line.points = [
                Point(x=positions[robot_id][0], y=positions[robot_id][1], z=0.08),
                Point(x=goal_x, y=goal_y, z=0.08),
            ]
            output.markers.append(line)
        self.marker_pub.publish(output)

    def _plan(self) -> None:
        if self.map_message is None or len(self.esdf_messages) != len(self.robot_ids):
            self._publish_status({'state': 'WAITING_FOR_MAPS'})
            return
        if self.map_message.header.frame_id != self.global_frame:
            self._publish_status(
                {'state': 'REJECTED_MAP_FRAME', 'frame_id': self.map_message.header.frame_id}
            )
            return
        try:
            for robot_id in self.robot_ids:
                message = self.esdf_messages[robot_id]
                if message.header.frame_id != self.global_frame:
                    raise ValueError(f'{robot_id} ESDF is outside {self.global_frame}')
            occupancy = self._grid_array(self.map_message)
            geometry = self._geometry(self.map_message)
            esdf = {}
            for robot_id in self.robot_ids:
                message = self.esdf_messages[robot_id]
                esdf[robot_id] = align_grid_to_geometry(
                    self._grid_array(message),
                    self._geometry(message),
                    occupancy.shape,
                    geometry,
                    fill_value=LETHAL_COST,
                )
            positions = self._robot_positions()
            starts = {
                robot_id: world_to_cell(*positions[robot_id], geometry)
                for robot_id in self.robot_ids
            }
            frontiers, _ = detect_frontiers(
                occupancy,
                starts.values(),
                geometry.resolution,
                float(self.get_parameter('min_frontier_length_m').value),
                occupied_threshold=OCCUPIED_THRESHOLD,
            )
            # Match the commissioned explorer's WFD reachability semantics:
            # frontier allocation follows known-free cells in /swarm/map.
            # The per-robot A* planner remains the authority that applies its
            # ESDF clearance gate and rejects an unsafe goal.
            occupancy_costs = np.zeros_like(occupancy, dtype=np.int16)
            distances = {
                robot_id: path_costs(
                    occupancy,
                    occupancy_costs,
                    starts[robot_id],
                    geometry.resolution,
                    occupied_threshold=OCCUPIED_THRESHOLD,
                    lethal_cost=LETHAL_COST,
                )
                for robot_id in self.robot_ids
            }
            standoff = float(
                self.get_parameter('frontier_standoff_distance_m').value
            )
            assignments = assign_frontiers(
                distances, frontiers,
                standoff_distance_m=standoff,
                resolution=geometry.resolution,
            )
            stamp = self.get_clock().now().to_msg()
            status_assignments = {}
            navigation_dispatch = self.dispatch_enabled and self.exploration_enabled
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if navigation_dispatch:
                self._feedback_events(positions, now_s)
                self._apply_goal_exclusions(distances, frontiers, geometry, now_s)
                idle_distances = {
                    robot_id: distances[robot_id]
                    for robot_id in self.robot_ids
                    if not self.progress.has_goal(robot_id)
                }
                assignments = assign_frontiers(
                    idle_distances,
                    frontiers,
                    min_path_cost_m=self.goal_tolerance_m,
                    standoff_distance_m=standoff,
                    resolution=geometry.resolution,
                )
                for robot_id in self.robot_ids:
                    active = self.progress.goal(robot_id)
                    if active is not None:
                        status_assignments[robot_id] = {
                            'goal_xy': [round(active[0], 3), round(active[1], 3)],
                            'state': 'active',
                        }
            else:
                assignments = assign_frontiers(
                    distances,
                    frontiers,
                    min_path_cost_m=self.goal_tolerance_m,
                    standoff_distance_m=standoff,
                    resolution=geometry.resolution,
                )
            for robot_id, assignment in assignments.items():
                goal_xy = cell_to_world(assignment.goal_cell, geometry)
                pose = self._pose(positions[robot_id], goal_xy, stamp)
                self.preview_pubs[robot_id].publish(pose)
                if navigation_dispatch and self.progress.set_goal(
                    robot_id, goal_xy, positions[robot_id], now_s
                ):
                    self.goal_pubs[robot_id].publish(pose)
                status_assignments[robot_id] = {
                    'frontier_index': assignment.frontier_index,
                    'goal_xy': [round(goal_xy[0], 3), round(goal_xy[1], 3)],
                    'path_cost_m': round(assignment.path_cost_m, 3),
                    'state': 'dispatched' if navigation_dispatch else 'preview',
                }
            self._publish_markers(frontiers, assignments, positions, geometry, stamp)
            self._publish_status(
                {
                    'state': 'READY',
                    'frontier_count': len(frontiers),
                    'assignments': status_assignments,
                    'last_events': self.last_events,
                    'exploration_enabled': self.exploration_enabled,
                    'navigation_dispatch': navigation_dispatch,
                }
            )
        except (ValueError, TransformException) as error:
            self._publish_status({'state': 'WAITING_OR_REJECTED', 'reason': str(error)})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CooperativeWfd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
