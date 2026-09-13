import math

import numpy as np

from aaa_navigation.kettle_navigation_geometry import approach_pose, transform_point


def test_transform_point_applies_rotation_and_translation():
    half = math.sqrt(0.5)
    result = transform_point([1.0, 0.0, 0.2], [2.0, 3.0, 0.5], [0, 0, half, half])
    assert np.allclose(result, [2.0, 4.0, 0.7])


def test_approach_pose_keeps_standoff_and_faces_target():
    x, y, yaw, distance = approach_pose([1.0, 2.0], [4.0, 6.0], 0.6)
    assert np.isclose(distance, 5.0)
    assert np.isclose(math.hypot(4.0 - x, 6.0 - y), 0.6)
    assert np.isclose(yaw, math.atan2(4.0, 3.0))


def test_approach_pose_does_not_drive_when_already_inside_standoff():
    x, y, _, _ = approach_pose([1.0, 2.0], [1.2, 2.0], 0.6)
    assert np.allclose([x, y], [1.0, 2.0])
