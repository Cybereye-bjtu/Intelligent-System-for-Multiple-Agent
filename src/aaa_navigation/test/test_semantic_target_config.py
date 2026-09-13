import json

import pytest

from aaa_navigation.semantic_target_config import (
    load_topology_objects_file,
    parse_semantic_targets,
    parse_topology_objects,
)


def test_structured_targets_generate_prompt_and_node_class():
    result = parse_semantic_targets(json.dumps({"semantic_targets": [{
        "target": "box", "attributes": ["white"],
        "text": "school", "relations": [],
    }]}))
    assert result[0]["prompt"] == "white box labeled school"
    assert result[0]["semantic_class"] == "school"


def test_single_object_and_string_attribute_are_accepted():
    result = parse_semantic_targets({
        "target": "box", "attributes": "white", "text": "bank", "relations": []
    })
    assert result[0]["prompt"] == "white box labeled bank"


@pytest.mark.parametrize("relations", [None, {}, ["near door"]])
def test_rejects_malformed_relations(relations):
    with pytest.raises(ValueError, match="relation"):
        parse_semantic_targets({
            "target": "box", "attributes": ["white"],
            "text": "school", "relations": relations,
        })


def test_supported_perception_relation_is_preserved():
    result = parse_semantic_targets({
        "target": "box", "attributes": ["white"], "text": "school",
        "relations": [{"type": "right_of", "reference": "door"}],
    })
    assert result[0]["relations"] == [{
        "type": "right_of", "reference": "door", "frame": "image",
    }]


def test_topology_objects_are_loaded_from_json_file(tmp_path):
    source = tmp_path / "objects.json"
    source.write_text(json.dumps({"version": 1, "topology_objects": [{
        "target": "door", "attributes": ["red"],
        "text": "", "relations": [],
    }]}), encoding="utf-8")

    result = load_topology_objects_file(source)

    assert result[0]["prompt"] == "red door"
    assert result[0]["semantic_class"] == "door"


def test_topology_object_file_requires_all_four_fields(tmp_path):
    source = tmp_path / "objects.json"
    source.write_text(json.dumps({"topology_objects": [{
        "target": "door", "attributes": ["red"], "relations": [],
    }]}), encoding="utf-8")

    with pytest.raises(ValueError, match="missing required fields: text"):
        load_topology_objects_file(source)


def test_topology_object_configuration_version_is_validated():
    with pytest.raises(ValueError, match="unsupported.*version"):
        parse_topology_objects({"version": 2, "topology_objects": [{
            "target": "door", "attributes": [], "text": "", "relations": [],
        }]})
