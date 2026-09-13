import pytest

from aaa_navigation.semantic_detection_contract import (
    legacy_prompt_query,
    normalize_detection_query,
    topology_detection_query,
)


def test_topology_query_preserves_all_semantic_fields():
    result = topology_detection_query({
        "target": "sign",
        "attributes": ["white"],
        "text": "BANK",
        "relations": [{"type": "right_of", "reference": "door"}],
        "semantic_class": "bank",
    })
    assert result["query_id"] == "topology:bank"
    assert result["purpose"] == "topology"
    assert result["target"] == "sign"
    assert result["attributes"] == ["white"]
    assert result["text_constraints"] == ["BANK"]
    assert result["ocr_target"] == "BANK"
    assert result["relations"] == [{"type": "right_of", "reference": "door"}]


def test_natural_language_compiler_shape_is_normalized():
    result = normalize_detection_query({
        "query_id": "mission:123",
        "purpose": "mission",
        "target": "sign",
        "attributes": "white sign",
        "text_constraints": ["BANK"],
        "relations": [],
    })
    assert result["attributes"] == ["white sign"]
    assert result["semantic_class"] == "sign"


def test_legacy_prompt_remains_a_supported_sam3_ocr_query():
    assert legacy_prompt_query("bank")["ocr_target"] == "bank"


def test_invalid_purpose_is_rejected():
    with pytest.raises(ValueError, match="purpose"):
        normalize_detection_query({"target": "bank", "purpose": "unknown"})
