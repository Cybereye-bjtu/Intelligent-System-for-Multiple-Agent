"""One-shot Jetson003 MQTT RGB-D + SAM3 target localization worker."""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shlex
import sys

import cv2
import numpy as np


def load_shell_environment(path):
    values = {}
    for raw_line in Path(path).read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[7:].strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = shlex.split(value.strip())[0] if value.strip() else ""
    return values


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = (float(value) for value in pose)
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [x, y, z]
    return matrix


def robust_position(mask, depth_m, intrinsics, camera_to_robot):
    valid = mask & np.isfinite(depth_m) & (depth_m >= 0.15) & (depth_m <= 8.0)
    rows, cols = np.where(valid)
    if rows.size < 80:
        raise RuntimeError(f"only {rows.size} valid target depth pixels")
    depth = depth_m[rows, cols].astype(np.float64)
    median = float(np.median(depth))
    mad = float(np.median(np.abs(depth - median)))
    keep = np.abs(depth - median) <= max(2.8 * 1.4826 * mad, 0.04)
    rows, cols, depth = rows[keep], cols[keep], depth[keep]
    intrinsic = np.asarray(intrinsics, dtype=float)
    x = (cols - intrinsic[0, 2]) * depth / intrinsic[0, 0]
    y = (rows - intrinsic[1, 2]) * depth / intrinsic[1, 1]
    camera = np.stack([x, y, depth], axis=1)
    robot = (camera_to_robot @ np.c_[camera, np.ones(len(camera))].T).T[:, :3]
    return np.median(camera, axis=0), np.median(robot, axis=0), len(depth)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", required=True)
    parser.add_argument("--artifact-dir", required=True)
    parser.add_argument("--black-root", required=True)
    parser.add_argument("--detector-root", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--mqtt-env-file", required=True)
    parser.add_argument("--broker", required=True)
    parser.add_argument("--port", type=int, default=8883)
    parser.add_argument("--device-id", default="jetson003")
    parser.add_argument("--world-frame", default="odom")
    parser.add_argument("--robot-frame", default="base_link")
    parser.add_argument("--prompt", default="white box")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()
    result_path = Path(args.result)
    try:
        # paho-mqtt is a pure-Python system package on the inference server.
        system_packages = "/usr/lib/python3/dist-packages"
        if system_packages not in sys.path:
            sys.path.append(system_packages)
        sys.path.insert(0, str(Path(args.black_root).resolve()))
        from hardware.mqtt_rgbd_agent import MqttRgbdAgent, MqttRgbdConfig

        environment = load_shell_environment(args.mqtt_env_file)
        username = environment.get("HYZX_MQTT_USERNAME", "")
        password = environment.get("HYZX_MQTT_PASSWORD", "")
        if not username or not password:
            raise RuntimeError("Jetson MQTT credentials are missing from the environment file")
        config = MqttRgbdConfig(
            broker=args.broker,
            port=args.port,
            username=username,
            password=password,
            device_id=args.device_id,
            world_frame=args.world_frame,
            robot_frame=args.robot_frame,
            capture_timeout_s=args.timeout,
            command_timeout_s=args.timeout + 8.0,
        )
        with MqttRgbdAgent(config) as agent:
            frame = agent.capture_metric_rgbd(retries=2)

        detector_root = str(Path(args.detector_root).resolve())
        sys.path.insert(0, detector_root)
        # Avoid resolving the black project's different perception package.
        for key in list(sys.modules):
            if key == "perception" or key.startswith("perception."):
                del sys.modules[key]
        from perception.sam3_detector import SAM3Detector

        lock_path = "/tmp/aaa_sam3_gpu.lock"
        with open(lock_path, "w", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            detector = SAM3Detector(
                checkpoint_path=args.checkpoint,
                device="cuda:0",
                confidence_threshold=args.confidence,
            )
            detections = detector.detect(frame["rgb"], args.prompt)
        if not detections:
            raise RuntimeError(f"SAM3 did not detect {args.prompt!r}")
        detection = max(detections, key=lambda item: item["confidence"])
        mask = np.asarray(detection["mask"], dtype=bool)
        if mask.shape != frame["depth_m"].shape:
            raise RuntimeError("SAM3 mask and aligned depth dimensions differ")
        world_to_camera = np.asarray(frame["camera_to_world"], dtype=float)
        world_to_robot = pose_matrix(frame["robot_pose"])
        camera_to_robot = np.linalg.inv(world_to_robot) @ world_to_camera
        camera, robot, pixels = robust_position(
            mask, frame["depth_m"], frame["intrinsics"], camera_to_robot
        )
        artifact_dir = Path(args.artifact_dir)
        artifact_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(artifact_dir / "latest_rgb.jpg"), frame["rgb"])
        cv2.imwrite(str(artifact_dir / "latest_mask.png"), mask.astype(np.uint8) * 255)
        result = {
            "success": True,
            "target": args.prompt,
            "confidence": float(detection["confidence"]),
            "bbox_xyxy": [float(value) for value in detection["bbox_xyxy"]],
            "valid_depth_pixels": int(pixels),
            "position_camera_optical_m": camera.tolist(),
            "position_robot_m": robot.tolist(),
            "robot_frame": args.robot_frame,
            "capture_world_frame": frame.get("world_frame", ""),
            "timestamp_ns": int(frame.get("timestamp_ns", 0)),
        }
    except Exception as error:
        result = {"success": False, "reason": str(error)}
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return 0 if result["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
