"""Validation helpers for the HYZX MQTT JSON envelope."""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Iterable, Tuple


class EnvelopeError(ValueError):
    """Raised when an MQTT telemetry envelope is malformed."""


def normalize_epoch_seconds(value: Any) -> float:
    """Normalize Unix seconds, milliseconds, microseconds, or nanoseconds."""
    try:
        timestamp = float(value)
    except (TypeError, ValueError) as error:
        raise EnvelopeError("timestamp must be numeric") from error
    if not math.isfinite(timestamp) or timestamp < 0.0:
        raise EnvelopeError("timestamp must be finite and non-negative")
    if timestamp >= 1e18:
        return timestamp / 1e9
    if timestamp >= 1e15:
        return timestamp / 1e6
    if timestamp >= 1e11:
        return timestamp / 1e3
    return timestamp


def decode_envelope(payload: bytes) -> Tuple[int, float, str, Any]:
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise EnvelopeError(f"invalid JSON: {error}") from error
    if not isinstance(message, dict):
        raise EnvelopeError("envelope must be an object")
    data = message.get("data")
    if not isinstance(data, (dict, list)):
        raise EnvelopeError("envelope data must be an object or array")
    try:
        sequence = int(message["seq"])
        timestamp = normalize_epoch_seconds(message["ts"])
    except (KeyError, TypeError, ValueError) as error:
        raise EnvelopeError("envelope requires numeric seq and ts") from error
    source = str(message.get("source", ""))
    return sequence, timestamp, source, data


def decode_stamp(stamp: Any) -> Tuple[int, int]:
    if isinstance(stamp, (int, float)):
        try:
            timestamp = normalize_epoch_seconds(stamp)
        except EnvelopeError:
            return 0, 0
        seconds = int(math.floor(timestamp))
        nanoseconds = int(round((timestamp - seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        return seconds, nanoseconds
    if isinstance(stamp, (list, tuple)) and len(stamp) >= 2:
        stamp = {"sec": stamp[0], "nanosec": stamp[1]}
    if not isinstance(stamp, dict):
        return 0, 0
    seconds = stamp.get("sec", stamp.get("secs", 0))
    nanoseconds = stamp.get("nanosec", stamp.get("nsecs", 0))
    try:
        seconds = int(seconds)
        nanoseconds = int(nanoseconds)
    except (TypeError, ValueError):
        return 0, 0
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return 0, 0
    return seconds, nanoseconds


def normalize_frame(frame: Any) -> str:
    return str(frame or "").lstrip("/")


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        output = float(value)
    except (TypeError, ValueError):
        return default
    return output if math.isfinite(output) else default


def finite_ranges(values: Iterable[Any], scale: float = 1.0) -> list:
    output = []
    for value in values or []:
        if value is None:
            output.append(math.inf)
            continue
        try:
            number = float(value) * scale
        except (TypeError, ValueError):
            number = math.inf
        output.append(number)
    return output


def compact_vector(values: Any, length: int, default: float = 0.0) -> list:
    """Decode the numeric arrays used by the HYZX compact MQTT schema."""
    output = [default] * length
    if not isinstance(values, (list, tuple)):
        return output
    for index, value in enumerate(values[:length]):
        output[index] = finite_float(value, default)
    return output


def scan_increment_from_endpoints(
    angle_min: float,
    angle_max: float,
    reading_count: int,
) -> float:
    """Recover precision lost when the compact angle fields are rounded."""
    if reading_count <= 1 or angle_max <= angle_min:
        return 0.0
    return (angle_max - angle_min) / (reading_count - 1)


def resample_scan_ranges(values: Iterable[Any], target_count: int) -> list:
    """Nearest-angle resampling for variable-beam rotating lidar scans."""
    source = list(values or [])
    if target_count <= 0 or not source:
        return []
    if len(source) == target_count:
        return source
    if target_count == 1:
        return [source[0]]
    if len(source) == 1:
        return source * target_count
    scale = (len(source) - 1) / (target_count - 1)
    return [source[round(index * scale)] for index in range(target_count)]


def consistent_scan_angle_max(
    angle_min: float,
    angle_increment: float,
    reading_count: int,
) -> float:
    """Return an inclusive angle_max matching the actual range array."""
    if reading_count <= 0 or angle_increment <= 0.0:
        return angle_min
    return angle_min + (reading_count - 1) * angle_increment


def fixed_length_floats(values: Any, length: int) -> list:
    if not isinstance(values, list) or len(values) != length:
        return [0.0] * length
    return [finite_float(value) for value in values]


def twist_publish_command(linear_x: Any, angular_z: Any) -> Dict[str, Any]:
    """Encode the generic ROS publish command accepted by the HYZX edge."""
    return {
        "cmd": "publish",
        "topic": "/cmd_vel",
        "msg_type": "geometry_msgs/Twist",
        "data": {
            "linear": {"x": finite_float(linear_x)},
            "angular": {"z": finite_float(angular_z)},
        },
    }


def simple_twist_command(linear_x: Any, angular_z: Any) -> Dict[str, float]:
    """Encode the compact velocity payload used by the jetson003 edge."""
    return {
        "linear_x": finite_float(linear_x),
        "angular_z": finite_float(angular_z),
    }
