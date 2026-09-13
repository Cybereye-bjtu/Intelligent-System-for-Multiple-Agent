import json

import pytest

from aaa_search_manager.target_observation_contract import load_target_observation


def valid_document():
    return {
        'success': True,
        'target_observation': {
            'schema': 'cybereye.target_observation.v1',
            'mission_id': 'mission-001',
            'robot_id': 'jetson003',
            'target_label': 'red cup',
            'instance_id': 'cup-1',
            'frame_id': 'base_link',
            'position': [1.2, -0.1, 0.4],
            'position_covariance': [
                0.01, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.03,
            ],
            'semantic_confidence': 0.91,
            'valid_depth_ratio': 0.72,
            'observation_count': 2,
        },
    }


def write_document(tmp_path, document):
    path = tmp_path / 'target_pose_result.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    return path


def test_load_normalizes_raw_vehicle_base_frame(tmp_path):
    value = load_target_observation(str(write_document(tmp_path, valid_document())))
    assert value['frame_id'] == 'jetson003/base_link'
    assert value['mission_id'] == 'mission-001'


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('mission_id', ''),
        ('robot_id', 'unknown'),
        ('frame_id', 'odom'),
        ('schema', 'other'),
    ],
)
def test_load_rejects_invalid_contract(tmp_path, field, value):
    document = valid_document()
    document['target_observation'][field] = value
    with pytest.raises(ValueError):
        load_target_observation(str(write_document(tmp_path, document)))
