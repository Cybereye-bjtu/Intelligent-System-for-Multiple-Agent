"""Pure geometry helpers for RGB-D target navigation."""

import math

import numpy as np


def quaternion_rotation_matrix(x, y, z, w):
    quaternion = np.asarray([x, y, z, w], dtype=float)
    norm = float(np.dot(quaternion, quaternion))
    if norm < 1e-12:
        raise ValueError("transform quaternion has zero length")
    x, y, z, w = quaternion / math.sqrt(norm)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_point(position, translation, quaternion):
    """Apply target<-source transform to a point in source coordinates."""
    point = np.asarray(position[:3], dtype=float)
    offset = np.asarray(translation[:3], dtype=float)
    return quaternion_rotation_matrix(*quaternion) @ point + offset


def approach_pose(robot_xy, target_xy, standoff_distance):
    """Return an approach point on the robot-to-target ray and target-facing yaw."""
    robot = np.asarray(robot_xy[:2], dtype=float)
    target = np.asarray(target_xy[:2], dtype=float)
    delta = target - robot
    distance = float(np.linalg.norm(delta))
    if distance < 1e-6:
        raise ValueError("target is coincident with the robot")
    standoff = max(0.0, float(standoff_distance))
    travel = max(0.0, distance - standoff)
    goal = robot + delta * (travel / distance)
    yaw = math.atan2(delta[1], delta[0])
    return float(goal[0]), float(goal[1]), float(yaw), distance
