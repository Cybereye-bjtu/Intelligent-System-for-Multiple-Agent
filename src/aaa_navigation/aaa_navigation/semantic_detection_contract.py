"""JSON-safe contracts for structured SAM3+OCR detection requests."""

from __future__ import annotations

from collections.abc import Mapping, Sequence


def normalize_detection_query(value, *, default_purpose="mission", default_id=""):
    if not isinstance(value, Mapping):
        raise TypeError("detection query must be an object")
    target = str(value.get("target") or "").strip()
    if not target:
        raise ValueError("detection query target is required")

    attributes = value.get("attributes", [])
    if isinstance(attributes, str):
        attributes = [attributes]
    if not isinstance(attributes, Sequence):
        raise TypeError("detection query attributes must be a string or list")
    attributes = [str(item).strip() for item in attributes if str(item).strip()]

    text_constraints = value.get("text_constraints")
    if text_constraints is None:
        legacy_text = str(value.get("text") or "").strip()
        text_constraints = [legacy_text] if legacy_text else []
    elif isinstance(text_constraints, str):
        text_constraints = [text_constraints]
    if not isinstance(text_constraints, Sequence):
        raise TypeError("detection query text_constraints must be a string or list")
    text_constraints = [
        str(item).strip() for item in text_constraints if str(item).strip()
    ]

    relations = value.get("relations", [])
    if not isinstance(relations, list):
        raise TypeError("detection query relations must be a list")
    query_id = str(value.get("query_id") or default_id or target).strip()
    purpose = str(value.get("purpose") or default_purpose).strip().lower()
    if purpose not in {"mission", "topology"}:
        raise ValueError("detection query purpose must be 'mission' or 'topology'")
    visual_prompt = str(value.get("visual_prompt") or target).strip()
    ocr_target = str(
        value.get("ocr_target")
        or (text_constraints[0] if text_constraints else target)
    ).strip()
    aliases = value.get("ocr_aliases", [])
    if isinstance(aliases, str):
        aliases = [aliases]
    if not isinstance(aliases, Sequence):
        raise TypeError("detection query ocr_aliases must be a string or list")

    return {
        "query_id": query_id,
        "purpose": purpose,
        "target": target,
        "attributes": attributes,
        "text_constraints": text_constraints,
        "relations": relations,
        "visual_prompt": visual_prompt,
        "ocr_target": ocr_target,
        "ocr_aliases": [str(item).strip() for item in aliases if str(item).strip()],
        "semantic_class": str(value.get("semantic_class") or target).strip(),
    }


def topology_detection_query(item):
    semantic_class = str(item.get("semantic_class") or item.get("text") or item["target"])
    return normalize_detection_query(
        {
            **dict(item),
            "query_id": f"topology:{semantic_class}",
            "purpose": "topology",
            "semantic_class": semantic_class,
        }
    )


def legacy_prompt_query(prompt):
    text = str(prompt or "").strip()
    if not text:
        raise ValueError("target prompt is empty")
    return normalize_detection_query(
        {"query_id": f"mission:{text}", "purpose": "mission", "target": text}
    )
