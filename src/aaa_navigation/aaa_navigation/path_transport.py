"""ROS-independent helpers for compact, bounded path transport."""

from __future__ import annotations

import math


def transform_xy(x, y, tx, ty, yaw):
    """Apply a planar source-to-target transform to one point."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        tx + cosine * float(x) - sine * float(y),
        ty + sine * float(x) + cosine * float(y),
    )


def encode_path(points, frame_id, sequence, timestamp, max_points=2000):
    if not frame_id:
        raise ValueError("path frame must not be empty")
    if len(points) > int(max_points):
        raise ValueError("path exceeds configured point limit")
    encoded = []
    for point in points:
        if len(point) != 2:
            raise ValueError("each path point must contain x and y")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("path contains a non-finite coordinate")
        encoded.append([round(x, 4), round(y, 4)])
    return {
        "v": 1,
        "seq": int(sequence),
        "ts": float(timestamp),
        "frame": str(frame_id),
        "points": encoded,
    }


def decode_path(payload, expected_frame, max_points=2000):
    if not isinstance(payload, dict) or int(payload.get("v", 0)) != 1:
        raise ValueError("unsupported path payload")
    frame = str(payload.get("frame", ""))
    if frame != str(expected_frame):
        raise ValueError("unexpected path frame")
    sequence = int(payload.get("seq", -1))
    timestamp = float(payload.get("ts", 0.0))
    raw_points = payload.get("points")
    if sequence < 0 or not math.isfinite(timestamp) or timestamp <= 0.0:
        raise ValueError("invalid path metadata")
    if not isinstance(raw_points, list) or len(raw_points) > int(max_points):
        raise ValueError("invalid path point count")
    points = []
    for point in raw_points:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError("invalid path point")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("path contains a non-finite coordinate")
        points.append((x, y))
    return sequence, timestamp, points
