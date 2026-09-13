import math

import numpy as np
import pytest

from aaa_search_manager.approach_planner import (
    generate_approach_goal,
    generate_approach_pair,
)


def grid(costs):
    return {'costs': costs, 'origin_x': -5.0, 'origin_y': -5.0, 'resolution': 0.1}


def test_generates_reachable_separated_goals_facing_target():
    costs = np.zeros((100, 100), dtype=np.int16)
    result = generate_approach_pair(
        target_xy=(0.0, 0.0),
        robot_positions={'hyzx001': (-2.0, 0.4), 'jetson003': (2.0, 0.0)},
        costmaps={'hyzx001': grid(costs), 'jetson003': grid(costs)},
    )
    first, second = result.values()
    assert math.hypot(first[0] - second[0], first[1] - second[1]) >= 0.3
    for x, y, yaw in result.values():
        assert math.isclose(math.hypot(x, y), 0.3, abs_tol=1e-9)
        expected_yaw = math.atan2(-y, -x)
        assert math.cos(yaw) == pytest.approx(math.cos(expected_yaw))
        assert math.sin(yaw) == pytest.approx(math.sin(expected_yaw))


def test_rejects_when_one_robot_has_no_reachable_candidate():
    free = np.zeros((100, 100), dtype=np.int16)
    blocked = np.full((100, 100), 100, dtype=np.int16)
    with pytest.raises(ValueError, match='jetson003'):
        generate_approach_pair(
            target_xy=(0.0, 0.0),
            robot_positions={'hyzx001': (-2.0, 0.4), 'jetson003': (2.0, 0.0)},
            costmaps={'hyzx001': grid(free), 'jetson003': grid(blocked)},
        )


def test_single_robot_fallback_uses_first_reachable_radius():
    costs = np.zeros((100, 100), dtype=np.int16)
    result = generate_approach_goal(
        target_xy=(0.0, 0.0),
        robot_position=(-2.0, 0.0),
        costmap=grid(costs),
        radii=(0.3, 0.5, 0.8),
    )
    assert math.hypot(result[0], result[1]) == pytest.approx(0.3)


def test_single_robot_fallback_honors_reserved_goal_separation():
    costs = np.zeros((100, 100), dtype=np.int16)
    result = generate_approach_goal(
        target_xy=(0.0, 0.0),
        robot_position=(2.0, 0.0),
        costmap=grid(costs),
        radii=(0.3,),
        reserved_poses=((0.3, 0.0),),
        min_separation=0.3,
    )
    assert math.hypot(result[0] - 0.3, result[1]) >= 0.3


def test_single_robot_fallback_rejects_lethal_start():
    costs = np.zeros((100, 100), dtype=np.int16)
    start_cell = (30, 50)
    costs[start_cell[1], start_cell[0]] = 100
    with pytest.raises(ValueError, match='no reachable'):
        generate_approach_goal(
            target_xy=(0.0, 0.0),
            robot_position=(-2.0, 0.0),
            costmap=grid(costs),
        )
