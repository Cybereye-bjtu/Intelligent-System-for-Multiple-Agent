import math

import numpy as np

from aaa_navigation.black_target_detection_worker import pose_matrix, robust_position


def test_pose_matrix_uses_degree_yaw():
    matrix = pose_matrix([1, 2, 3, 0, 0, 90])
    assert np.allclose(matrix[:3, 3], [1, 2, 3])
    assert np.allclose(matrix[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-7)


def test_robust_position_returns_robot_frame_median():
    depth = np.ones((20, 20), dtype=np.float32) * 2.0
    mask = np.ones_like(depth, dtype=bool)
    intrinsics = np.asarray([[10, 0, 10], [0, 10, 10], [0, 0, 1]], dtype=float)
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 3]
    camera, robot, count = robust_position(mask, depth, intrinsics, transform)
    assert count == 400
    assert np.allclose(robot, camera + [1, 2, 3])
