"""Validation and prompt generation for structured semantic-map targets."""
from __future__ import annotations

import json
from pathlib import Path


TOPOLOGY_OBJECT_FIELDS = frozenset({"target", "attributes", "text", "relations"})
SUPPORTED_RELATIONS = frozenset({
    "left_of", "right_of", "above", "below", "leftmost", "rightmost",
    "topmost", "bottommost", "nearest_to", "farthest_from",
})


def parse_topology_objects(document, *, require_all_fields=False):
    """Return normalized topology objects from JSON text or decoded JSON.

    ``relations`` are perception-time spatial constraints. They are distinct
    from topological graph edges, which still come only from robot traversal.
    """
    value = json.loads(document) if isinstance(document, str) else document
    if isinstance(value, dict) and (
        "topology_objects" in value or "semantic_targets" in value
    ):
        if "topology_objects" in value and "semantic_targets" in value:
            raise ValueError(
                "topology object configuration cannot contain both "
                "topology_objects and semantic_targets"
            )
        version = value.get("version", 1)
        if version != 1:
            raise ValueError(
                f"unsupported topology object configuration version: {version}"
            )
        value = value.get("topology_objects", value.get("semantic_targets"))
    elif isinstance(value, dict):
        value = [value]
    if not isinstance(value, list) or not value:
        raise ValueError("topology object configuration must contain a non-empty list")

    normalized = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"topology object {index} must be an object")
        if require_all_fields:
            missing = TOPOLOGY_OBJECT_FIELDS - set(item)
            if missing:
                raise ValueError(
                    f"topology object {index} is missing required fields: "
                    + ", ".join(sorted(missing))
                )
        target = str(item.get("target") or "").strip()
        text = str(item.get("text") or "").strip()
        attributes = item.get("attributes", [])
        relations = item.get("relations", [])
        if isinstance(attributes, str):
            attributes = [attributes]
        if not target or not isinstance(attributes, list):
            raise ValueError(f"topology object {index} has invalid target/attributes")
        attributes = [str(value).strip() for value in attributes if str(value).strip()]
        if not isinstance(relations, list):
            raise ValueError("relations must be a list")
        normalized_relations = []
        for relation in relations:
            if not isinstance(relation, dict):
                raise ValueError("every relation must be an object")
            relation_type = str(relation.get("type") or "").strip().lower()
            reference = str(relation.get("reference") or "").strip()
            if relation_type not in SUPPORTED_RELATIONS or not reference:
                raise ValueError(
                    f"unsupported or incomplete relation: {relation!r}"
                )
            expected_frame = (
                "world" if relation_type in {"nearest_to", "farthest_from"}
                else "image"
            )
            frame = str(relation.get("frame") or expected_frame).strip().lower()
            if frame != expected_frame:
                raise ValueError(
                    f"relation {relation_type} must use frame {expected_frame}"
                )
            normalized_relations.append({
                "type": relation_type,
                "reference": reference,
                "frame": frame,
            })
        description = " ".join([*attributes, target])
        prompt = f"{description} labeled {text}" if text else description
        normalized.append({
            "target": target,
            "attributes": attributes,
            "text": text,
            "text_constraints": [text] if text else [],
            "relations": normalized_relations,
            # Retained only for status/UI compatibility. Detection consumes the
            # structured fields and never reparses this display string.
            "prompt": prompt,
            "semantic_class": text or target,
        })
    prompts = [item["prompt"] for item in normalized]
    if len(prompts) != len(set(prompts)):
        raise ValueError("topology object prompts must be unique")
    return normalized


def load_topology_objects_file(path):
    """Load and validate the persistent topology-object dictionary."""
    source = Path(path).expanduser()
    if not source.is_file():
        raise ValueError(f"topology object file does not exist: {source}")
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except OSError as error:
        raise ValueError(
            f"cannot read topology object file {source}: {error}"
        ) from error
    except json.JSONDecodeError as error:
        raise ValueError(
            f"invalid JSON in topology object file {source}: {error}"
        ) from error
    return parse_topology_objects(document, require_all_fields=True)


def parse_semantic_targets(document):
    """Compatibility alias for the former configuration API."""
    return parse_topology_objects(document)
