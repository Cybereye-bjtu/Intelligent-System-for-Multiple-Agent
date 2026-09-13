"""One-shot MQTT RGB-D + SAM3 kettle localization worker.

This module runs in the dedicated SAM3 Python environment.  It writes its
machine-readable result to a file so ROS logs from model loading cannot corrupt
the protocol used by the parent ROS node.
"""

import argparse
import fcntl
import json
import math
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np


def capture_timestamp_ns(metadata):
    """Return the RGB acquisition time carried by the synchronized snapshot."""
    timestamp = float(metadata.get("ts", 0.0))
    if not math.isfinite(timestamp) or timestamp <= 0.0:
        raise ValueError("RGB-D snapshot is missing a valid capture timestamp")
    return int(round(timestamp * 1_000_000_000))


def latest_capture(directory):
    metadata = sorted(
        directory.glob("frame_*_camera.json"), key=lambda path: path.stat().st_mtime
    )
    if not metadata:
        raise RuntimeError("MQTT snapshot did not create RGB-D files")
    metadata_path = metadata[-1]
    stem = metadata_path.name.removesuffix("_camera.json")
    return (
        directory / f"{stem}.jpg",
        directory / f"{stem}_depth_m.npy",
        metadata_path,
    )


def robust_position(mask, depth_m, intrinsics, transform):
    valid = mask & np.isfinite(depth_m) & (depth_m >= 0.15) & (depth_m <= 5.0)
    rows, cols = np.where(valid)
    if rows.size < 80:
        raise RuntimeError(f"only {rows.size} valid target depth pixels")
    depth = depth_m[rows, cols].astype(np.float64)
    median = float(np.median(depth))
    mad = float(np.median(np.abs(depth - median)))
    keep = np.abs(depth - median) <= max(2.8 * 1.4826 * mad, 0.04)
    rows, cols, depth = rows[keep], cols[keep], depth[keep]
    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    camera = np.stack(
        [(cols - cx) * depth / fx, (rows - cy) * depth / fy, depth], axis=1
    )
    robot = (transform @ np.c_[camera, np.ones(len(camera))].T).T[:, :3]
    return np.median(camera, axis=0), np.median(robot, axis=0), len(depth)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", required=True)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--cybereye-root", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--camera-to-robot", required=True)
    parser.add_argument("--prompt", default="white box")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    result_path = Path(args.result)
    try:
        root = Path(args.cybereye_root).resolve()
        sys.path.insert(0, str(root))
        capture_dir = Path(args.capture_dir)
        capture_dir.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                "/usr/bin/python3", str(root / "camera_view.py"), "--rgbd",
                "--snapshot", "--save", str(capture_dir), "--no-window",
                "--timeout", str(args.timeout),
            ],
            check=True,
            timeout=args.timeout + 10.0,
        )
        image_path, depth_path, metadata_path = latest_capture(capture_dir)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("captured RGB image is unreadable")
        depth_m = np.load(depth_path)
        metadata = json.loads(metadata_path.read_text())
        from perception.sam3_detector import SAM3Detector

        with open("/tmp/aaa_sam3_gpu.lock", "w", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            detector = SAM3Detector(
                checkpoint_path=args.checkpoint,
                device="cuda:0",
                confidence_threshold=args.confidence,
            )
            detections = detector.detect(image, args.prompt)
        if not detections:
            raise RuntimeError(f"SAM3 did not detect {args.prompt!r}")
        detection = max(detections, key=lambda item: item["confidence"])
        mask = np.asarray(detection["mask"], dtype=bool)
        if mask.shape != depth_m.shape:
            raise RuntimeError("SAM3 mask and aligned depth dimensions differ")
        transform_value = json.loads(Path(args.camera_to_robot).read_text())
        transform = np.asarray(
            transform_value.get("camera_to_robot", transform_value), dtype=float
        ).reshape(4, 4)
        camera, robot, pixels = robust_position(
            mask, depth_m, metadata["intrinsics"], transform
        )
        result = {
            "success": True,
            "target": args.prompt,
            "confidence": float(detection["confidence"]),
            "bbox_xyxy": [float(value) for value in detection["bbox_xyxy"]],
            "valid_depth_pixels": int(pixels),
            "position_camera_optical_m": camera.tolist(),
            "position_robot_m": robot.tolist(),
            "robot_frame": "base_footprint",
            "timestamp_ns": capture_timestamp_ns(metadata),
            "rgb_path": str(image_path),
            "depth_path": str(depth_path),
        }
    except Exception as error:
        result = {"success": False, "reason": str(error)}
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return 0 if result["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
