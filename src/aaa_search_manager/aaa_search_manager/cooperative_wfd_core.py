"""Pure cooperative Wavefront Frontier Detection and assignment helpers.

The detector follows the WFD structure from Keidar and Kaminka: start in the
robots' reachable known-free space, search that space once, and group frontier
cells on the boundary with unknown space.  Assignment is a one-to-one minimum
path-cost matching; for the two-robot AAA fleet it is solved exactly by small
enumeration rather than adding an optimization dependency.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


Cell = Tuple[int, int]  # row, column
NEIGHBORS_4 = ((-1, 0), (1, 0), (0, -1), (0, 1))
NEIGHBORS_8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


@dataclass(frozen=True)
class GridGeometry:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class Frontier:
    cells: Tuple[Cell, ...]
    length_m: float
    centroid: Tuple[float, float]  # row, column in continuous grid coordinates


@dataclass(frozen=True)
class FrontierAssignment:
    robot_id: str
    frontier_index: int
    goal_cell: Cell
    path_cost_m: float


@dataclass
class _TrackedGoal:
    goal_xy: Tuple[float, float]
    best_distance_m: float
    last_progress_s: float


class GoalProgressTracker:
    """Small, ROS-independent arrival/stall monitor for dispatched WFD goals."""

    def __init__(self, goal_tolerance_m: float, progress_timeout_s: float) -> None:
        self.goal_tolerance_m = max(0.0, float(goal_tolerance_m))
        self.progress_timeout_s = max(0.1, float(progress_timeout_s))
        self._goals: Dict[str, _TrackedGoal] = {}

    def clear(self, robot_id: Optional[str] = None) -> None:
        if robot_id is None:
            self._goals.clear()
        else:
            self._goals.pop(robot_id, None)

    def has_goal(self, robot_id: str) -> bool:
        return robot_id in self._goals

    def goal(self, robot_id: str) -> Optional[Tuple[float, float]]:
        tracked = self._goals.get(robot_id)
        return None if tracked is None else tracked.goal_xy

    def set_goal(
        self,
        robot_id: str,
        goal_xy: Tuple[float, float],
        robot_xy: Tuple[float, float],
        now_s: float,
    ) -> bool:
        """Track a goal and return true only when it materially changed."""
        old = self._goals.get(robot_id)
        if old is not None and math.dist(old.goal_xy, goal_xy) <= self.goal_tolerance_m:
            return False
        distance = math.dist(robot_xy, goal_xy)
        self._goals[robot_id] = _TrackedGoal(
            goal_xy=(float(goal_xy[0]), float(goal_xy[1])),
            best_distance_m=distance,
            last_progress_s=float(now_s),
        )
        return True

    def update(
        self,
        robot_positions: Mapping[str, Tuple[float, float]],
        now_s: float,
    ) -> Dict[str, str]:
        """Return and remove goals that were reached or made no progress."""
        events: Dict[str, str] = {}
        # One map cell at the deployed resolution is meaningful progress; a
        # fixed 5 cm deadband avoids adding another commissioning parameter.
        progress_deadband_m = 0.05
        for robot_id, tracked in list(self._goals.items()):
            if robot_id not in robot_positions:
                continue
            distance = math.dist(robot_positions[robot_id], tracked.goal_xy)
            if distance <= self.goal_tolerance_m:
                events[robot_id] = 'reached'
                del self._goals[robot_id]
                continue
            if tracked.best_distance_m - distance >= progress_deadband_m:
                tracked.best_distance_m = distance
                tracked.last_progress_s = float(now_s)
            elif float(now_s) - tracked.last_progress_s >= self.progress_timeout_s:
                events[robot_id] = 'stalled'
                del self._goals[robot_id]
        return events


def _inside(shape: Sequence[int], row: int, column: int) -> bool:
    return 0 <= row < int(shape[0]) and 0 <= column < int(shape[1])


def world_to_cell(x: float, y: float, geometry: GridGeometry) -> Cell:
    """Convert a world point to a grid cell, supporting a rotated grid origin."""
    if geometry.resolution <= 0.0:
        raise ValueError("grid resolution must be positive")
    dx = float(x) - geometry.origin_x
    dy = float(y) - geometry.origin_y
    cosine = math.cos(geometry.origin_yaw)
    sine = math.sin(geometry.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (
        int(math.floor(local_y / geometry.resolution)),
        int(math.floor(local_x / geometry.resolution)),
    )


def cell_to_world(cell: Cell, geometry: GridGeometry) -> Tuple[float, float]:
    """Return the world position at the center of a grid cell."""
    row, column = cell
    local_x = (float(column) + 0.5) * geometry.resolution
    local_y = (float(row) + 0.5) * geometry.resolution
    cosine = math.cos(geometry.origin_yaw)
    sine = math.sin(geometry.origin_yaw)
    return (
        geometry.origin_x + cosine * local_x - sine * local_y,
        geometry.origin_y + sine * local_x + cosine * local_y,
    )


def align_grid_to_geometry(
    source: np.ndarray,
    source_geometry: GridGeometry,
    target_shape: Sequence[int],
    target_geometry: GridGeometry,
    fill_value: int = 100,
) -> np.ndarray:
    """Sample a source grid at target-cell centers using world coordinates.

    AAA's ESDF mapper deliberately pads `/swarm/map`; therefore equal frame IDs
    do not imply equal array indices.  This conversion keeps WFD on the raw map
    while applying the deployed ESDF safety gate at the correct world cells.
    """
    if source.ndim != 2:
        raise ValueError("source grid must be two-dimensional")
    height, width = int(target_shape[0]), int(target_shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("target shape must be positive")
    rows, columns = np.indices((height, width), dtype=np.float64)
    target_x = (columns + 0.5) * target_geometry.resolution
    target_y = (rows + 0.5) * target_geometry.resolution
    target_cosine = math.cos(target_geometry.origin_yaw)
    target_sine = math.sin(target_geometry.origin_yaw)
    world_x = (
        target_geometry.origin_x
        + target_cosine * target_x
        - target_sine * target_y
    )
    world_y = (
        target_geometry.origin_y
        + target_sine * target_x
        + target_cosine * target_y
    )
    dx = world_x - source_geometry.origin_x
    dy = world_y - source_geometry.origin_y
    source_cosine = math.cos(source_geometry.origin_yaw)
    source_sine = math.sin(source_geometry.origin_yaw)
    source_x = source_cosine * dx + source_sine * dy
    source_y = -source_sine * dx + source_cosine * dy
    source_columns = np.floor(source_x / source_geometry.resolution).astype(np.int64)
    source_rows = np.floor(source_y / source_geometry.resolution).astype(np.int64)
    inside = (
        (source_rows >= 0)
        & (source_rows < source.shape[0])
        & (source_columns >= 0)
        & (source_columns < source.shape[1])
    )
    output = np.full((height, width), int(fill_value), dtype=source.dtype)
    output[inside] = source[source_rows[inside], source_columns[inside]]
    return output


def nearest_free_cell(free: np.ndarray, start: Cell) -> Optional[Cell]:
    """Find the nearest known-free cell to a possibly noisy robot grid cell."""
    if free.ndim != 2 or not free.size:
        return None
    row = min(max(int(start[0]), 0), free.shape[0] - 1)
    column = min(max(int(start[1]), 0), free.shape[1] - 1)
    queue = deque([(row, column)])
    visited = np.zeros(free.shape, dtype=bool)
    visited[row, column] = True
    while queue:
        current = queue.popleft()
        if bool(free[current]):
            return current
        for dr, dc in NEIGHBORS_4:
            nr, nc = current[0] + dr, current[1] + dc
            if _inside(free.shape, nr, nc) and not visited[nr, nc]:
                visited[nr, nc] = True
                queue.append((nr, nc))
    return None


def reachable_known_free(
    occupancy: np.ndarray,
    starts: Iterable[Cell],
    occupied_threshold: int = 65,
) -> Tuple[np.ndarray, Tuple[Cell, ...]]:
    """Multi-source WFD outer search over the union of robot-reachable space."""
    if occupancy.ndim != 2:
        raise ValueError("occupancy grid must be two-dimensional")
    free = (occupancy >= 0) & (occupancy < int(occupied_threshold))
    seeds: List[Cell] = []
    for start in starts:
        seed = nearest_free_cell(free, start)
        if seed is not None and seed not in seeds:
            seeds.append(seed)
    reachable = np.zeros(free.shape, dtype=bool)
    queue = deque(seeds)
    for seed in seeds:
        reachable[seed] = True
    while queue:
        row, column = queue.popleft()
        for dr, dc in NEIGHBORS_4:
            nr, nc = row + dr, column + dc
            if (
                _inside(free.shape, nr, nc)
                and free[nr, nc]
                and not reachable[nr, nc]
            ):
                reachable[nr, nc] = True
                queue.append((nr, nc))
    return reachable, tuple(seeds)


def _frontier_mask(occupancy: np.ndarray, reachable: np.ndarray) -> np.ndarray:
    unknown = occupancy < 0
    adjacent_unknown = np.zeros(unknown.shape, dtype=bool)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    return reachable & adjacent_unknown


def detect_frontiers(
    occupancy: np.ndarray,
    starts: Iterable[Cell],
    resolution: float,
    min_frontier_length_m: float,
    occupied_threshold: int = 65,
) -> Tuple[List[Frontier], Tuple[Cell, ...]]:
    """Detect and 8-connectivity-cluster reachable free frontier cells."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    reachable, seeds = reachable_known_free(
        occupancy, starts, occupied_threshold=occupied_threshold
    )
    mask = _frontier_mask(occupancy, reachable)
    visited = np.zeros(mask.shape, dtype=bool)
    frontiers: List[Frontier] = []
    minimum = max(0.0, float(min_frontier_length_m))
    for row, column in np.argwhere(mask):
        row, column = int(row), int(column)
        if visited[row, column]:
            continue
        queue = deque([(row, column)])
        visited[row, column] = True
        cells: List[Cell] = []
        while queue:
            cell = queue.popleft()
            cells.append(cell)
            for dr, dc in NEIGHBORS_8:
                nr, nc = cell[0] + dr, cell[1] + dc
                if (
                    _inside(mask.shape, nr, nc)
                    and mask[nr, nc]
                    and not visited[nr, nc]
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        length_m = len(cells) * float(resolution)
        if length_m + 1e-12 < minimum:
            continue
        values = np.asarray(cells, dtype=np.float64)
        centroid = tuple(float(value) for value in np.mean(values, axis=0))
        frontiers.append(
            Frontier(cells=tuple(cells), length_m=length_m, centroid=centroid)
        )
    frontiers.sort(key=lambda item: (-item.length_m, item.centroid))
    return frontiers, seeds


def path_costs(
    occupancy: np.ndarray,
    esdf_costs: np.ndarray,
    start: Cell,
    resolution: float,
    occupied_threshold: int = 65,
    lethal_cost: int = 100,
) -> np.ndarray:
    """Dijkstra path lengths through known, non-lethal cells.

    ESDF is used as a safety gate while the metric remains path length.  This
    keeps the initial deployment free of gain/clearance weighting parameters.
    """
    if occupancy.shape != esdf_costs.shape:
        raise ValueError("occupancy and ESDF grids must have identical shapes")
    known_free = (occupancy >= 0) & (occupancy < int(occupied_threshold))
    traversable = known_free & (esdf_costs >= 0) & (esdf_costs < int(lethal_cost))
    seed = nearest_free_cell(traversable, start)
    distances = np.full(occupancy.shape, np.inf, dtype=np.float64)
    if seed is None:
        return distances
    distances[seed] = 0.0
    queue: List[Tuple[float, int, int]] = [(0.0, seed[0], seed[1])]
    while queue:
        distance, row, column = heapq.heappop(queue)
        if distance != distances[row, column]:
            continue
        for dr, dc in NEIGHBORS_8:
            nr, nc = row + dr, column + dc
            if not _inside(traversable.shape, nr, nc) or not traversable[nr, nc]:
                continue
            step = float(resolution) * (math.sqrt(2.0) if dr and dc else 1.0)
            candidate = distance + step
            if candidate < distances[nr, nc]:
                distances[nr, nc] = candidate
                heapq.heappush(queue, (candidate, nr, nc))
    return distances


def _frontier_options(
    robot_id: str,
    distances: np.ndarray,
    frontiers: Sequence[Frontier],
    min_path_cost_m: float,
    standoff_distance_m: float,
    resolution: float,
) -> Dict[int, FrontierAssignment]:
    options: Dict[int, FrontierAssignment] = {}
    for index, frontier in enumerate(frontiers):
        best_cell = min(frontier.cells, key=lambda cell: distances[cell])
        goal_cell = best_cell
        steps = int(math.ceil(max(0.0, standoff_distance_m) / resolution))
        for _ in range(steps):
            row, column = goal_cell
            candidates = [
                (row + dr, column + dc)
                for dr, dc in NEIGHBORS_8
                if _inside(distances.shape, row + dr, column + dc)
                and distances[row + dr, column + dc] < distances[goal_cell]
            ]
            if not candidates:
                break
            goal_cell = min(candidates, key=lambda cell: distances[cell])
        cost = float(distances[goal_cell])
        if math.isfinite(cost) and cost > float(min_path_cost_m):
            options[index] = FrontierAssignment(
                robot_id=robot_id,
                frontier_index=index,
                goal_cell=goal_cell,
                path_cost_m=cost,
            )
    return options


def assign_frontiers(
    robot_distances: Mapping[str, np.ndarray],
    frontiers: Sequence[Frontier],
    min_path_cost_m: float = 0.0,
    standoff_distance_m: float = 0.0,
    resolution: float = 1.0,
) -> Dict[str, FrontierAssignment]:
    """Return the exact one-to-one assignment with maximum robot coverage.

    The primary objective assigns as many robots as possible.  The secondary
    objective minimizes total path length, followed by larger frontier length
    as a deterministic tie breaker.
    """
    robot_ids = tuple(robot_distances)
    if not robot_ids or not frontiers:
        return {}
    options = {
        robot_id: _frontier_options(
            robot_id,
            robot_distances[robot_id],
            frontiers,
            min_path_cost_m,
            standoff_distance_m,
            resolution,
        )
        for robot_id in robot_ids
    }
    choices = [tuple([None, *sorted(options[robot_id])]) for robot_id in robot_ids]
    best = None
    best_score = None
    for selected in itertools.product(*choices):
        assigned_indices = [value for value in selected if value is not None]
        if len(assigned_indices) != len(set(assigned_indices)):
            continue
        assignments = {
            robot_id: options[robot_id][frontier_index]
            for robot_id, frontier_index in zip(robot_ids, selected)
            if frontier_index is not None
        }
        total_cost = sum(value.path_cost_m for value in assignments.values())
        total_length = sum(
            frontiers[value.frontier_index].length_m
            for value in assignments.values()
        )
        index_tie = tuple(
            len(frontiers) if value is None else int(value) for value in selected
        )
        score = (-len(assignments), total_cost, -total_length, index_tie)
        if best_score is None or score < best_score:
            best_score = score
            best = assignments
    return best or {}
