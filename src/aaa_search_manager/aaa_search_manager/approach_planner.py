"""Map-backed generation of separated per-robot target approach poses."""
from __future__ import annotations

import heapq
import math
from typing import Dict, Iterable, Mapping, Sequence, Tuple

import numpy as np


def world_to_cell(x, y, *, origin_x, origin_y, resolution):
    return (
        int(math.floor((float(x) - origin_x) / resolution)),
        int(math.floor((float(y) - origin_y) / resolution)),
    )


def _reachable_distances(costs, start, goals, lethal_cost):
    height, width = costs.shape
    sx, sy = start
    if not (0 <= sx < width and 0 <= sy < height) or costs[sy, sx] >= lethal_cost:
        return {}
    remaining = set(goals)
    distances = np.full((height, width), np.inf, dtype=np.float64)
    distances[sy, sx] = 0.0
    queue = [(0.0, sx, sy)]
    found = {}
    moves = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
    )
    while queue and remaining:
        distance, x, y = heapq.heappop(queue)
        if distance != distances[y, x]:
            continue
        cell = (x, y)
        if cell in remaining:
            found[cell] = distance
            remaining.remove(cell)
        for dx, dy, step in moves:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            cost = int(costs[ny, nx])
            if cost >= lethal_cost:
                continue
            candidate = distance + step * (1.0 + 0.02 * max(cost, 0))
            if candidate < distances[ny, nx]:
                distances[ny, nx] = candidate
                heapq.heappush(queue, (candidate, nx, ny))
    return found


def generate_approach_goal(
    *,
    target_xy: Sequence[float],
    robot_position: Sequence[float],
    costmap: Mapping[str, object],
    radii: Iterable[float] = (0.3, 0.5, 0.8),
    angle_samples: int = 24,
    max_goal_cost: int = 75,
    lethal_cost: int = 96,
    reserved_poses: Iterable[Sequence[float]] = (),
    min_separation: float = 0.30,
) -> Tuple[float, float, float]:
    """Return one reachable goal, trying radii in order.

    ``reserved_poses`` keeps a later sequential goal separated from robots that
    have already been assigned an approach pose.
    """
    target_x, target_y = (float(value) for value in target_xy[:2])
    costs = np.asarray(costmap['costs'], dtype=np.int16)
    if costs.ndim != 2:
        raise ValueError('costmap must be two-dimensional')
    metadata = {
        'origin_x': float(costmap['origin_x']),
        'origin_y': float(costmap['origin_y']),
        'resolution': float(costmap['resolution']),
    }
    start = world_to_cell(*robot_position[:2], **metadata)
    reserved = [tuple(float(value) for value in pose[:2]) for pose in reserved_poses]
    generated = False
    for radius in radii:
        radius = float(radius)
        if radius <= 0.0:
            continue
        generated = True
        cell_to_candidates = {}
        for index in range(max(4, int(angle_samples))):
            angle = 2.0 * math.pi * index / max(4, int(angle_samples))
            x = target_x + radius * math.cos(angle)
            y = target_y + radius * math.sin(angle)
            if any(math.hypot(x - rx, y - ry) < min_separation for rx, ry in reserved):
                continue
            yaw = math.atan2(target_y - y, target_x - x)
            cell = world_to_cell(x, y, **metadata)
            cx, cy = cell
            if (
                0 <= cy < costs.shape[0]
                and 0 <= cx < costs.shape[1]
                and int(costs[cy, cx]) <= max_goal_cost
            ):
                cell_to_candidates.setdefault(cell, []).append((x, y, yaw))
        distances = _reachable_distances(costs, start, cell_to_candidates, lethal_cost)
        options = []
        for cell, path_cost in distances.items():
            for candidate in cell_to_candidates[cell]:
                options.append((path_cost, candidate))
        if options:
            return min(options, key=lambda item: item[0])[1]
    if not generated:
        raise ValueError('no approach candidates were generated')
    raise ValueError('no reachable approach pose')


def generate_approach_pair(
    *,
    target_xy: Sequence[float],
    robot_positions: Mapping[str, Sequence[float]],
    costmaps: Mapping[str, Mapping[str, object]],
    radii: Iterable[float] = (0.3, 0.5, 0.8),
    angle_samples: int = 24,
    max_goal_cost: int = 75,
    lethal_cost: int = 96,
    min_pair_separation: float = 0.30,
) -> Dict[str, Tuple[float, float, float]]:
    robot_ids = list(robot_positions)
    if len(robot_ids) != 2 or set(robot_ids) != set(costmaps):
        raise ValueError('exactly two matching robot positions and costmaps are required')
    radii = tuple(float(radius) for radius in radii if float(radius) > 0.0)
    if len(radii) > 1:
        last_error = None
        for radius in radii:
            try:
                return generate_approach_pair(
                    target_xy=target_xy,
                    robot_positions=robot_positions,
                    costmaps=costmaps,
                    radii=(radius,),
                    angle_samples=angle_samples,
                    max_goal_cost=max_goal_cost,
                    lethal_cost=lethal_cost,
                    min_pair_separation=min_pair_separation,
                )
            except ValueError as error:
                last_error = error
        raise last_error
    target_x, target_y = (float(value) for value in target_xy[:2])
    world_candidates = []
    for radius in radii:
        if radius <= 0.0:
            continue
        for index in range(max(4, int(angle_samples))):
            angle = 2.0 * math.pi * index / max(4, int(angle_samples))
            x = target_x + float(radius) * math.cos(angle)
            y = target_y + float(radius) * math.sin(angle)
            yaw = math.atan2(target_y - y, target_x - x)
            world_candidates.append((x, y, yaw))
    if not world_candidates:
        raise ValueError('no approach candidates were generated')

    ranked = {}
    for robot_id in robot_ids:
        grid = costmaps[robot_id]
        costs = np.asarray(grid['costs'], dtype=np.int16)
        if costs.ndim != 2:
            raise ValueError(f'{robot_id} costmap must be two-dimensional')
        metadata = {
            'origin_x': float(grid['origin_x']),
            'origin_y': float(grid['origin_y']),
            'resolution': float(grid['resolution']),
        }
        start = world_to_cell(*robot_positions[robot_id][:2], **metadata)
        cell_to_candidates = {}
        for candidate in world_candidates:
            cell = world_to_cell(candidate[0], candidate[1], **metadata)
            x, y = cell
            if (
                0 <= y < costs.shape[0]
                and 0 <= x < costs.shape[1]
                and int(costs[y, x]) <= max_goal_cost
            ):
                cell_to_candidates.setdefault(cell, []).append(candidate)
        distances = _reachable_distances(costs, start, cell_to_candidates, lethal_cost)
        options = []
        for cell, path_cost in distances.items():
            for candidate in cell_to_candidates[cell]:
                options.append((path_cost, candidate))
        ranked[robot_id] = sorted(options, key=lambda item: item[0])
        if not ranked[robot_id]:
            raise ValueError(f'no reachable approach pose for {robot_id}')

    best = None
    first_id, second_id = robot_ids
    for first_cost, first_pose in ranked[first_id]:
        for second_cost, second_pose in ranked[second_id]:
            separation = math.hypot(
                first_pose[0] - second_pose[0], first_pose[1] - second_pose[1]
            )
            if separation < min_pair_separation:
                continue
            score = first_cost + second_cost - 0.1 * min(separation, 2.0)
            if best is None or score < best[0]:
                best = (score, first_pose, second_pose)
    if best is None:
        raise ValueError('no reachable approach-pose pair satisfies separation')
    return {first_id: best[1], second_id: best[2]}
