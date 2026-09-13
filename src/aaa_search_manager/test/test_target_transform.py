import math

import numpy as np
import pytest

from aaa_search_manager.target_transform import (
    transform_point_and_covariance,
    validate_observation_values,
)


def test_transform_point_and_covariance_rotates_into_site_map():
    yaw = math.pi / 2.0
    point, covariance = transform_point_and_covariance(
        [1.0, 0.0, 0.0],
        [0.04, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.09],
        [2.0, 3.0, 0.0],
        [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)],
    )
    assert point == pytest.approx([2.0, 4.0, 0.0])
    assert covariance == pytest.approx(np.diag([0.01, 0.04, 0.09]))


def test_quality_gate_accepts_valid_observation():
    point, covariance = validate_observation_values(
        position=[1.0, 2.0, 0.5],
        covariance=np.eye(3).reshape(-1),
        semantic_confidence=0.9,
        valid_depth_ratio=0.8,
        observation_count=2,
        min_semantic_confidence=0.8,
        min_valid_depth_ratio=0.35,
        min_observation_count=2,
    )
    assert point.tolist() == [1.0, 2.0, 0.5]
    assert covariance.shape == (3, 3)


@pytest.mark.parametrize(
    'overrides',
    [
        {'semantic_confidence': 0.2},
        {'valid_depth_ratio': 0.1},
        {'observation_count': 1},
        {'position': [float('nan'), 0.0, 0.0]},
        {'covariance': [1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
    ],
)
def test_quality_gate_rejects_bad_observation(overrides):
    values = {
        'position': [1.0, 2.0, 0.5],
        'covariance': np.eye(3).reshape(-1),
        'semantic_confidence': 0.9,
        'valid_depth_ratio': 0.8,
        'observation_count': 2,
    }
    values.update(overrides)
    with pytest.raises(ValueError):
        validate_observation_values(
            **values,
            min_semantic_confidence=0.8,
            min_valid_depth_ratio=0.35,
            min_observation_count=2,
        )
