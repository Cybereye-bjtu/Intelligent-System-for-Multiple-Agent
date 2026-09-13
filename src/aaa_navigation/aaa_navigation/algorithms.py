"""ROS-independent geometry, distance-field, A*, and controller helpers."""

from __future__ import annotations

import heapq
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


GridPoint = Tuple[int, int]
Point2D = Tuple[float, float]


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def transform_polar_points(
    ranges: np.ndarray,
    angles: np.ndarray,
    translation_x: float,
    translation_y: float,
    yaw: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Transform finite polar points into another planar coordinate frame."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = np.asarray(angles, dtype=np.float64)
    if ranges.shape != angles.shape:
        raise ValueError("ranges and angles must have the same shape")
    source_x = ranges * np.cos(angles)
    source_y = ranges * np.sin(angles)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    target_x = translation_x + cosine * source_x - sine * source_y
    target_y = translation_y + sine * source_x + cosine * source_y
    return np.hypot(target_x, target_y), np.arctan2(target_y, target_x)


def world_to_grid(
    x: float, y: float, origin_x: float, origin_y: float, resolution: float
) -> GridPoint:
    return (
        int(math.floor((x - origin_x) / resolution)),
        int(math.floor((y - origin_y) / resolution)),
    )


def grid_to_world(
    gx: int, gy: int, origin_x: float, origin_y: float, resolution: float
) -> Point2D:
    return (
        origin_x + (gx + 0.5) * resolution,
        origin_y + (gy + 0.5) * resolution,
    )


def _edt_1d(values: np.ndarray) -> np.ndarray:
    """Felzenszwalb-Huttenlocher squared Euclidean distance transform."""
    n = int(values.shape[0])
    result = np.full(n, np.inf, dtype=np.float64)
    finite = np.flatnonzero(np.isfinite(values))
    if finite.size == 0:
        return result

    v = np.empty(n, dtype=np.int32)
    z = np.empty(n + 1, dtype=np.float64)
    k = 0
    v[0] = int(finite[0])
    z[0] = -np.inf
    z[1] = np.inf

    for q in finite[1:]:
        q = int(q)
        while True:
            vk = int(v[k])
            s = ((values[q] + q * q) - (values[vk] + vk * vk)) / (
                2.0 * (q - vk)
            )
            if s > z[k] or k == 0:
                break
            k -= 1
        if s <= z[k]:
            k = 0
        else:
            k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = np.inf

    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        delta = q - int(v[k])
        result[q] = delta * delta + values[int(v[k])]
    return result


def euclidean_distance_transform(obstacles: np.ndarray) -> np.ndarray:
    """Return distance in cells to True obstacle cells."""
    if obstacles.ndim != 2:
        raise ValueError("obstacles must be a 2-D array")
    height, width = obstacles.shape
    source = np.where(obstacles, 0.0, np.inf)
    temporary = np.empty_like(source, dtype=np.float64)
    result = np.empty_like(source, dtype=np.float64)
    for y in range(height):
        temporary[y, :] = _edt_1d(source[y, :])
    for x in range(width):
        result[:, x] = _edt_1d(temporary[:, x])
    return np.sqrt(result)


def _octile(dx: int, dy: int) -> float:
    diagonal = min(dx, dy)
    return math.sqrt(2.0) * diagonal + max(dx, dy) - diagonal


def nearest_traversable(
    costs: np.ndarray,
    point: GridPoint,
    lethal_cost: int,
    max_radius: int = 30,
) -> Optional[GridPoint]:
    """Project a pose in an inflated cell onto the closest traversable cell."""
    height, width = costs.shape
    px, py = point

    def valid(x: int, y: int) -> bool:
        return (
            0 <= x < width
            and 0 <= y < height
            and int(costs[y, x]) < lethal_cost
        )

    if valid(px, py):
        return point
    for radius in range(1, max_radius + 1):
        candidates = []
        for y in range(py - radius, py + radius + 1):
            for x in (px - radius, px + radius):
                if valid(x, y):
                    candidates.append((x, y))
        for x in range(px - radius + 1, px + radius):
            for y in (py - radius, py + radius):
                if valid(x, y):
                    candidates.append((x, y))
        if candidates:
            return min(
                candidates,
                key=lambda item: (item[0] - px) ** 2 + (item[1] - py) ** 2,
            )
    return None


def astar_grid(
    costs: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    lethal_cost: int = 96,
    allow_diagonal: bool = True,
    clearance_weight: float = 2.5,
    turn_weight: float = 0.1,
    max_expansions: int = 500000,
) -> List[GridPoint]:
    """Plan on an OccupancyGrid-like cost array; points are (x, y)."""
    height, width = costs.shape

    def valid(point: GridPoint) -> bool:
        x, y = point
        return (
            0 <= x < width
            and 0 <= y < height
            and int(costs[y, x]) < lethal_cost
        )

    start = nearest_traversable(costs, start, lethal_cost)
    goal = nearest_traversable(costs, goal, lethal_cost)
    if start is None or goal is None:
        return []

    moves = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    if allow_diagonal:
        moves += [(1, 1), (-1, 1), (-1, -1), (1, -1)]

    # State includes the previous direction to support a small turn penalty.
    start_state = (start[0], start[1], 0, 0)
    queue = [(0.0, 0.0, start_state)]
    best = {start_state: 0.0}
    parent = {}
    goal_state = None
    expansions = 0

    while queue and expansions < max_expansions:
        _, current_g, state = heapq.heappop(queue)
        if current_g != best.get(state):
            continue
        x, y, prev_dx, prev_dy = state
        expansions += 1
        if (x, y) == goal:
            goal_state = state
            break

        for dx, dy in moves:
            nx, ny = x + dx, y + dy
            if not valid((nx, ny)):
                continue
            if dx and dy:
                # Forbid cutting diagonally through obstacle corners.
                if not valid((x + dx, y)) or not valid((x, y + dy)):
                    continue
            distance_cost = math.sqrt(2.0) if dx and dy else 1.0
            obstacle_cost = clearance_weight * (float(costs[ny, nx]) / 100.0) ** 2
            direction_changed = (prev_dx or prev_dy) and (dx, dy) != (
                prev_dx,
                prev_dy,
            )
            candidate_g = (
                current_g
                + distance_cost
                + obstacle_cost
                + (turn_weight if direction_changed else 0.0)
            )
            next_state = (nx, ny, dx, dy)
            if candidate_g >= best.get(next_state, np.inf):
                continue
            best[next_state] = candidate_g
            parent[next_state] = state
            heuristic = _octile(abs(goal[0] - nx), abs(goal[1] - ny))
            heapq.heappush(
                queue, (candidate_g + heuristic, candidate_g, next_state)
            )

    if goal_state is None:
        return []
    path = []
    state = goal_state
    while True:
        path.append((state[0], state[1]))
        if state == start_state:
            break
        state = parent[state]
    path.reverse()
    return path


def line_is_free(
    costs: np.ndarray, start: GridPoint, end: GridPoint, lethal_cost: int
) -> bool:
    """Integer supercover-like collision test between two grid cells."""
    x0, y0 = start
    x1, y1 = end
    steps = max(abs(x1 - x0), abs(y1 - y0))
    if steps == 0:
        return int(costs[y0, x0]) < lethal_cost
    for i in range(steps + 1):
        ratio = i / steps
        x = int(round(x0 + ratio * (x1 - x0)))
        y = int(round(y0 + ratio * (y1 - y0)))
        if int(costs[y, x]) >= lethal_cost:
            return False
    return True


def simplify_path(
    path: Sequence[GridPoint], costs: np.ndarray, lethal_cost: int
) -> List[GridPoint]:
    if len(path) <= 2:
        return list(path)
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if line_is_free(costs, path[anchor], path[candidate], lethal_cost):
                break
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified


def resample_polyline(
    points: Sequence[Point2D], spacing: float
) -> List[Point2D]:
    """Densify a polyline without moving or removing its corner points."""
    if len(points) <= 1 or spacing <= 0.0:
        return list(points)
    output = [points[0]]
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        steps = max(1, int(math.ceil(length / spacing)))
        for step in range(1, steps + 1):
            ratio = step / steps
            output.append(
                (start[0] + ratio * dx, start[1] + ratio * dy)
            )
    return output


def nearest_path_index(points: Sequence[Point2D], position: Point2D) -> int:
    return min(
        range(len(points)),
        key=lambda i: (points[i][0] - position[0]) ** 2
        + (points[i][1] - position[1]) ** 2,
    )


def _circumcenter(a: Point2D, b: Point2D, c: Point2D) -> Optional[Point2D]:
    ax, ay = a
    bx, by = b
    cx, cy = c
    denominator = 2.0 * (
        ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)
    )
    if abs(denominator) < 1e-8:
        return None
    aa = ax * ax + ay * ay
    bb = bx * bx + by * by
    cc = cx * cx + cy * cy
    return (
        (aa * (by - cy) + bb * (cy - ay) + cc * (ay - by))
        / denominator,
        (aa * (cx - bx) + bb * (ax - cx) + cc * (bx - ax))
        / denominator,
    )


def corner_aware_target(
    points: Sequence[Point2D],
    position: Point2D,
    lookahead: float,
    corner_search: float,
    corner_threshold: float,
    shift_gain: float,
    shift_max: float,
) -> Point2D:
    """Select a lookahead point and shift it to the outside of a sharp curve."""
    if not points:
        raise ValueError("path is empty")
    nearest = nearest_path_index(points, position)
    selected = len(points) - 1
    lookahead_selected = False
    travelled = 0.0
    previous = points[nearest]
    corner_index = None

    for i in range(nearest + 1, len(points)):
        segment = math.hypot(
            points[i][0] - previous[0], points[i][1] - previous[1]
        )
        travelled += segment
        previous = points[i]
        if 0 < i < len(points) - 1 and travelled <= corner_search:
            incoming = math.atan2(
                points[i][1] - points[i - 1][1],
                points[i][0] - points[i - 1][0],
            )
            outgoing = math.atan2(
                points[i + 1][1] - points[i][1],
                points[i + 1][0] - points[i][0],
            )
            if abs(wrap_angle(outgoing - incoming)) >= corner_threshold:
                corner_index = i
                break
        if travelled >= lookahead and not lookahead_selected:
            selected = i
            lookahead_selected = True
        if travelled > corner_search:
            break

    if corner_index is not None:
        selected = corner_index
    if selected <= 0 or selected >= len(points) - 1:
        return points[selected]

    center = _circumcenter(
        points[selected - 1], points[selected], points[selected + 1]
    )
    if center is None:
        return points[selected]
    dx = points[selected][0] - center[0]
    dy = points[selected][1] - center[1]
    radius = math.hypot(dx, dy)
    if radius < 1e-6:
        return points[selected]
    curvature = 1.0 / radius
    robot_distance = max(
        math.hypot(
            points[selected][0] - position[0],
            points[selected][1] - position[1],
        ),
        0.1,
    )
    shift = min(shift_max, shift_gain * curvature / robot_distance)
    return (
        points[selected][0] + shift * dx / radius,
        points[selected][1] + shift * dy / radius,
    )


def find_free_valleys(blocked: np.ndarray) -> List[Tuple[int, int]]:
    """Return inclusive runs of free sectors; histogram endpoints do not wrap."""
    valleys = []
    start = None
    for index, value in enumerate(blocked):
        if not value and start is None:
            start = index
        if value and start is not None:
            valleys.append((start, index - 1))
            start = None
    if start is not None:
        valleys.append((start, len(blocked) - 1))
    return valleys


def choose_vfh_heading(
    histogram: np.ndarray,
    observed: np.ndarray,
    target_angle: float,
    previous_angle: float,
    obstacle_threshold: float,
    sector_width: float,
    minimum_valley_width: int,
    margin_sectors: int,
    heading_weight: float,
    previous_weight: float,
    clearance_weight: float,
) -> Optional[float]:
    """Simplified two-step VFH* direction choice over feasible valleys."""
    blocked = np.logical_or(histogram >= obstacle_threshold, ~observed)
    valleys = [
        valley
        for valley in find_free_valleys(blocked)
        if valley[1] - valley[0] + 1 >= minimum_valley_width
    ]
    candidates = []
    sector_count = len(histogram)

    def sector_angle(index: int) -> float:
        return -math.pi + (index + 0.5) * sector_width

    for start, end in valleys:
        width = end - start + 1
        if width <= 2 * margin_sectors + 1:
            indices = [(start + end) // 2]
        else:
            safe_start = start + margin_sectors
            safe_end = end - margin_sectors
            target_index = int((wrap_angle(target_angle) + math.pi) / sector_width)
            target_index = max(safe_start, min(safe_end, target_index))
            indices = [safe_start, target_index, safe_end, (start + end) // 2]
        for index in set(indices):
            if not 0 <= index < sector_count:
                continue
            angle = sector_angle(index)
            clearance = max(0.0, 1.0 - float(histogram[index]))
            cost = (
                heading_weight * abs(wrap_angle(angle - target_angle))
                + previous_weight * abs(wrap_angle(angle - previous_angle))
                - clearance_weight * clearance
            )
            # A shallow second step penalizes headings that immediately point
            # toward a neighboring occupied sector.
            projected = int(
                round(index + 0.5 * wrap_angle(target_angle - angle) / sector_width)
            )
            projected = max(0, min(sector_count - 1, projected))
            if blocked[projected]:
                cost += heading_weight
            candidates.append((cost, angle))
    return min(candidates)[1] if candidates else None


def choose_vfh_recovery_heading(
    histogram: np.ndarray,
    observed: np.ndarray,
    target_angle: float,
    previous_angle: float,
    sector_width: float,
) -> Optional[float]:
    """Choose a least-dangerous observed direction for zero-speed recovery.

    This is used only when no valley is wide enough for forward motion.  It
    prevents a partial-FOV lidar from causing a blind, fixed +/-90 degree
    fallback rotation.
    """
    if len(histogram) == 0 or len(histogram) != len(observed):
        return None
    candidates = []
    for index in np.flatnonzero(observed):
        danger = float(histogram[index])
        if not math.isfinite(danger):
            continue
        angle = -math.pi + (int(index) + 0.5) * sector_width
        cost = (
            4.0 * danger
            + 2.0 * abs(wrap_angle(angle - target_angle))
            + abs(wrap_angle(angle - previous_angle))
        )
        candidates.append((cost, angle))
    return min(candidates)[1] if candidates else None


def fuzzy_velocity(
    front_distance: float,
    heading_error: float,
    max_linear: float,
    max_angular: float,
) -> Tuple[float, float]:
    """Small transparent fuzzy inference equivalent without scikit-fuzzy."""
    distance_near = np.clip((0.9 - front_distance) / 0.6, 0.0, 1.0)
    distance_far = np.clip((front_distance - 0.6) / 1.8, 0.0, 1.0)
    turn = np.clip(abs(heading_error) / (math.pi / 2.0), 0.0, 1.0)

    weights = np.array(
        [
            max(distance_near, turn),
            min(1.0 - distance_near, max(turn, 1.0 - distance_far)),
            min(distance_far, 1.0 - turn),
        ]
    )
    consequents = np.array([0.04, 0.45 * max_linear, max_linear])
    linear = (
        float(np.dot(weights, consequents) / np.sum(weights))
        if np.sum(weights) > 1e-9
        else 0.0
    )
    if abs(heading_error) > math.radians(80.0):
        linear = 0.0
    angular = max_angular * math.tanh(1.8 * heading_error)
    return linear, angular
