"""Pure target-observation validation and rigid-transform helpers."""
from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

import numpy as np


def validate_observation_values(
    *,
    position: Sequence[float],
    covariance: Sequence[float],
    semantic_confidence: float,
    valid_depth_ratio: float,
    observation_count: int,
    min_semantic_confidence: float,
    min_valid_depth_ratio: float,
    min_observation_count: int,
) -> Tuple[np.ndarray, np.ndarray]:
    point = np.asarray(position, dtype=float)
    cov = np.asarray(covariance, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("target position must contain three finite values")
    if cov.shape != (9,) or not np.all(np.isfinite(cov)):
        raise ValueError("target covariance must contain nine finite values")
    cov = cov.reshape(3, 3)
    if not np.allclose(cov, cov.T, atol=1e-8):
        raise ValueError("target covariance must be symmetric")
    if np.min(np.linalg.eigvalsh(cov)) < -1e-8:
        raise ValueError("target covariance must be positive semidefinite")
    confidence = float(semantic_confidence)
    depth_ratio = float(valid_depth_ratio)
    if not math.isfinite(confidence) or confidence < min_semantic_confidence:
        raise ValueError("semantic confidence is below the configured threshold")
    if not math.isfinite(depth_ratio) or depth_ratio < min_valid_depth_ratio:
        raise ValueError("valid-depth ratio is below the configured threshold")
    if int(observation_count) < int(min_observation_count):
        raise ValueError("observation count is below the configured threshold")
    return point, cov


def quaternion_rotation_matrix(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("transform quaternion is invalid")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def transform_point_and_covariance(
    position: Sequence[float],
    covariance: Sequence[float],
    translation_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray]:
    point = np.asarray(position, dtype=float).reshape(3)
    cov = np.asarray(covariance, dtype=float).reshape(3, 3)
    translation = np.asarray(translation_xyz, dtype=float).reshape(3)
    rotation = quaternion_rotation_matrix(quaternion_xyzw)
    return rotation @ point + translation, rotation @ cov @ rotation.T
