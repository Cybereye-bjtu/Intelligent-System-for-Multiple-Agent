import math

import pytest

from aaa_search_manager.approach_dispatch import order_by_assigned_goal_distance


def test_robot_nearer_its_assigned_goal_moves_first():
    sequence, distances = order_by_assigned_goal_distance(
        ["hyzx001", "jetson003"],
        {
            "hyzx001": (4.0, 0.0),
            "jetson003": (2.0, 0.0),
        },
        {
            "hyzx001": (0.0, 0.0),
            "jetson003": (1.5, 0.0),
        },
    )

    assert sequence == ["jetson003", "hyzx001"]
    assert math.isclose(distances["hyzx001"], 4.0)
    assert math.isclose(distances["jetson003"], 0.5)


def test_configured_order_breaks_equal_distance_ties():
    sequence, _ = order_by_assigned_goal_distance(
        ["hyzx001", "jetson003"],
        {
            "hyzx001": (1.0, 0.0),
            "jetson003": (3.0, 0.0),
        },
        {
            "hyzx001": (0.0, 0.0),
            "jetson003": (2.0, 0.0),
        },
    )

    assert sequence == ["hyzx001", "jetson003"]


def test_unassigned_robot_is_kept_second_for_safe_fallback():
    sequence, distances = order_by_assigned_goal_distance(
        ["hyzx001", "jetson003"],
        {"jetson003": (2.0, 0.0)},
        {"jetson003": (1.5, 0.0)},
    )

    assert sequence == ["jetson003", "hyzx001"]
    assert set(distances) == {"jetson003"}


def test_missing_position_is_rejected():
    with pytest.raises(ValueError, match="missing current position"):
        order_by_assigned_goal_distance(
            ["hyzx001", "jetson003"],
            {"hyzx001": (1.0, 0.0)},
            {},
        )
