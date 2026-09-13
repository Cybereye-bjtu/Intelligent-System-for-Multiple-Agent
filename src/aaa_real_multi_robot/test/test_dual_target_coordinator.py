import math

import pytest

from aaa_real_multi_robot.dual_target_coordinator import approach_goal


def test_shared_target_goal_uses_recipient_pose_and_standoff():
    x, y, yaw = approach_goal(0.0, 0.0, 2.0, 0.0, 0.30)
    assert x == pytest.approx(1.70)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_shared_target_goal_faces_target_from_other_side():
    x, y, yaw = approach_goal(2.0, 0.0, 0.0, 0.0, 0.30)
    assert x == pytest.approx(0.30)
    assert y == pytest.approx(0.0)
    assert abs(yaw) == pytest.approx(math.pi)


def test_shared_target_inside_standoff_is_rejected():
    with pytest.raises(ValueError):
        approach_goal(0.0, 0.0, 0.10, 0.0, 0.30)
