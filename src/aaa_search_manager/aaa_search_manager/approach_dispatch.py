"""Pure ordering rules for sequential two-robot target approach."""

from __future__ import annotations

import math
from typing import Mapping, Sequence


def order_by_assigned_goal_distance(
    navigation_order: Sequence[str],
    assigned_goals: Mapping[str, Sequence[float]],
    robot_positions: Mapping[str, Sequence[float]],
):
    """Return assigned robots nearest-first and unassigned robots last.

    The configured navigation order is only a deterministic tie-breaker. Each
    distance is from a robot's current position to that robot's assigned goal.
    """
    order = [str(robot_id) for robot_id in navigation_order]
    if len(order) != len(set(order)):
        raise ValueError("navigation_order contains duplicate robot IDs")
    unknown = set(assigned_goals) - set(order)
    if unknown:
        raise ValueError(f"assigned goals contain unknown robots: {sorted(unknown)}")

    distances = {}
    for robot_id in assigned_goals:
        if robot_id not in robot_positions:
            raise ValueError(f"missing current position for {robot_id}")
        goal = assigned_goals[robot_id]
        position = robot_positions[robot_id]
        if len(goal) < 2 or len(position) < 2:
            raise ValueError(f"goal and position for {robot_id} must be two-dimensional")
        values = tuple(float(value) for value in (*goal[:2], *position[:2]))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"goal or position for {robot_id} is non-finite")
        distances[robot_id] = math.hypot(
            values[0] - values[2], values[1] - values[3]
        )

    tie_breaker = {robot_id: index for index, robot_id in enumerate(order)}
    assigned = sorted(
        assigned_goals,
        key=lambda robot_id: (distances[robot_id], tie_breaker[robot_id]),
    )
    unassigned = [robot_id for robot_id in order if robot_id not in assigned_goals]
    return assigned + unassigned, distances
