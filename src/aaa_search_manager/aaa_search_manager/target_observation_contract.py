"""File boundary between the model virtualenv and the ROS 2 runtime."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from .target_transform import validate_observation_values


SCHEMA = 'cybereye.target_observation.v1'
BASE_FRAMES = {
    'hyzx001': 'hyzx001/base_footprint',
    'jetson003': 'jetson003/base_link',
}


def load_target_observation(path: str) -> Dict[str, Any]:
    document = json.loads(Path(path).expanduser().resolve().read_text(encoding='utf-8'))
    value = document.get('target_observation') if isinstance(document, dict) else None
    if not isinstance(value, dict):
        raise ValueError('result JSON does not contain target_observation')
    if value.get('schema') != SCHEMA:
        raise ValueError(f"unsupported target-observation schema: {value.get('schema')!r}")
    mission_id = str(value.get('mission_id') or '').strip()
    robot_id = str(value.get('robot_id') or '').strip()
    if not mission_id:
        raise ValueError('target observation has an empty mission_id')
    if robot_id not in BASE_FRAMES:
        raise ValueError(f'unsupported robot_id: {robot_id!r}')
    source_frame = str(value.get('frame_id') or '').strip().lstrip('/')
    expected_frame = BASE_FRAMES[robot_id]
    allowed_source_frames = {expected_frame, expected_frame.split('/', 1)[1]}
    if source_frame not in allowed_source_frames:
        raise ValueError(
            f'target observation frame {source_frame!r} is not the {robot_id} base frame'
        )
    point, covariance = validate_observation_values(
        position=value.get('position', []),
        covariance=value.get('position_covariance', []),
        semantic_confidence=value.get('semantic_confidence', 0.0),
        valid_depth_ratio=value.get('valid_depth_ratio', 0.0),
        observation_count=value.get('observation_count', 0),
        min_semantic_confidence=0.0,
        min_valid_depth_ratio=0.0,
        min_observation_count=1,
    )
    return {
        **value,
        'mission_id': mission_id,
        'robot_id': robot_id,
        'frame_id': expected_frame,
        'position': point.tolist(),
        'position_covariance': covariance.reshape(-1).tolist(),
    }
