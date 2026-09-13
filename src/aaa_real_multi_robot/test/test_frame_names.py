import math

import pytest

from aaa_real_multi_robot.frame_adapter import prefixed, relative_se2


def test_prefixes_bare_frame():
    assert prefixed("hyzx001", "base_footprint") == "hyzx001/base_footprint"


def test_does_not_double_prefix():
    assert prefixed("jetson003", "/jetson003/odom") == "jetson003/odom"


def test_empty_frame_stays_empty():
    assert prefixed("hyzx001", "") == ""


def test_initial_odometry_pose_is_zero():
    origin = (2.0, -3.0, math.radians(100.0))
    assert relative_se2(*origin, origin) == pytest.approx((0.0, 0.0, 0.0))


def test_odometry_motion_is_rotated_into_start_frame():
    origin = (2.0, -3.0, math.pi / 2.0)
    relative = relative_se2(2.0, -2.0, math.pi / 2.0, origin)
    assert relative == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
