"""Persistent RGB-D worker with peer SAM3 and PaddleOCR target discovery."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

from .black_target_detection_worker import (
    load_shell_environment,
    pose_matrix,
    robust_position as robust_black_position,
)
from .kettle_detection_worker import (
    capture_timestamp_ns,
    latest_capture,
    robust_position as robust_hyzx_position,
)
from .persistent_detection_protocol import (
    STARTUP_ERROR_FILE,
    atomic_write_json,
    serve_requests,
)
from .semantic_detection_contract import legacy_prompt_query, normalize_detection_query


class PersistentTargetEngine:
    def __init__(self, args):
        self.args = args
        self.agent = None
        detector_root = str(Path(args.detector_root).resolve())
        if detector_root not in sys.path:
            sys.path.insert(0, detector_root)
        for key in list(sys.modules):
            if key == "perception" or key.startswith("perception."):
                del sys.modules[key]
        from geometry.target_pose import TargetPoseConfig, TargetPoseEstimator
        from perception.ocr_detector import PaddleOCRDetector
        from perception.query_spec import TargetQuery
        from perception.sam3_detector import SAM3Detector
        from perception.semantic_fusion import fuse_sam3_ocr
        from perception.structured_semantic_pipeline import apply_structured_query

        self.lock_path = str(args.gpu_lock)
        with open(self.lock_path, "w", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            self.sam3 = SAM3Detector(
                checkpoint_path=args.checkpoint,
                device="cuda:0",
                confidence_threshold=args.confidence,
            )
            self.ocr = PaddleOCRDetector(
                detection_model_dir=args.ocr_detection_model,
                recognition_model_dir=args.ocr_recognition_model,
                device=args.ocr_device,
                ocr_version=args.ocr_version,
                min_recognition_score=args.ocr_confidence,
            )
        self.TargetQuery = TargetQuery
        self.fuse_sam3_ocr = fuse_sam3_ocr
        self.apply_structured_query = apply_structured_query
        self.pose_estimator = TargetPoseEstimator(
            TargetPoseConfig(
                min_depth_m=0.15,
                max_depth_m=8.0 if args.backend == "jetson_rpc" else 5.0,
                min_valid_pixels=80,
                min_valid_depth_ratio=0.0,
            )
        )
        self.attribute_matcher = None
        self.expected_instances = {}
        self.camera_to_robot = None
        self.mqtt_config = None
        if args.backend == "hyzx_snapshot":
            transform_value = json.loads(Path(args.camera_to_robot).read_text())
            self.camera_to_robot = np.asarray(
                transform_value.get("camera_to_robot", transform_value), dtype=float
            ).reshape(4, 4)
        else:
            self._prepare_jetson_config()
        self.last_visualization_path = ""

    def _visualization_path(self):
        directory = Path(
            self.args.capture_dir
            if self.args.backend == "hyzx_snapshot"
            else self.args.artifact_dir
        )
        directory.mkdir(parents=True, exist_ok=True)
        return directory / "latest_detection.jpg"

    def _write_visualization(self, image, prompt, detection=None, error=""):
        canvas = np.asarray(image).copy()
        if detection is not None:
            mask = np.asarray(detection["mask"], dtype=bool)
            if mask.shape == canvas.shape[:2]:
                overlay = canvas.copy()
                overlay[mask] = (0, 220, 0)
                canvas = cv2.addWeighted(canvas, 0.65, overlay, 0.35, 0.0)
            x1, y1, x2, y2 = [int(round(value)) for value in detection["bbox_xyxy"]]
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"DETECTED {prompt} {float(detection['confidence']):.2f}"
            color = (0, 255, 0)
        else:
            text = f"NOT DETECTED: {prompt}" if not error else f"ERROR: {error}"
            color = (0, 0, 255)
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 0), -1)
        cv2.putText(
            canvas, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2,
            cv2.LINE_AA,
        )
        output = self._visualization_path()
        temporary = output.with_name(output.stem + ".tmp" + output.suffix)
        if not cv2.imwrite(str(temporary), canvas):
            raise RuntimeError("failed to write target detection visualization")
        os.replace(temporary, output)
        self.last_visualization_path = str(output)
        return self.last_visualization_path

    def _prepare_jetson_config(self):
        system_packages = "/usr/lib/python3/dist-packages"
        if system_packages not in sys.path:
            sys.path.append(system_packages)
        black_root = str(Path(self.args.black_root).resolve())
        if black_root not in sys.path:
            sys.path.insert(0, black_root)
        from hardware.mqtt_rgbd_agent import MqttRgbdConfig

        environment = load_shell_environment(self.args.mqtt_env_file)
        username = environment.get("HYZX_MQTT_USERNAME", "")
        password = environment.get("HYZX_MQTT_PASSWORD", "")
        if not username or not password:
            raise RuntimeError("Jetson MQTT credentials are missing")
        self.mqtt_config = MqttRgbdConfig(
            broker=self.args.broker,
            port=self.args.port,
            username=username,
            password=password,
            device_id=self.args.device_id,
            world_frame=self.args.world_frame,
            robot_frame=self.args.robot_frame,
            capture_timeout_s=self.args.timeout,
            command_timeout_s=self.args.timeout + 8.0,
        )

    def _capture_hyzx(self):
        root = Path(self.args.cybereye_root).resolve()
        capture_dir = Path(self.args.capture_dir)
        capture_dir.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                "/usr/bin/python3",
                str(root / "camera_view.py"),
                "--rgbd",
                "--snapshot",
                "--save",
                str(capture_dir),
                "--no-window",
                "--timeout",
                str(self.args.timeout),
            ],
            check=True,
            timeout=self.args.timeout + 10.0,
        )
        image_path, depth_path, metadata_path = latest_capture(capture_dir)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("captured RGB image is unreadable")
        metadata = json.loads(metadata_path.read_text())
        return {
            "image": image,
            "depth": np.load(depth_path),
            "intrinsics": metadata["intrinsics"],
            "camera_to_robot": self.camera_to_robot,
            "timestamp_ns": capture_timestamp_ns(metadata),
            "rgb_path": str(image_path),
            "depth_path": str(depth_path),
        }

    def _get_agent(self):
        if self.agent is None:
            from hardware.mqtt_rgbd_agent import MqttRgbdAgent

            self.agent = MqttRgbdAgent(self.mqtt_config)
        return self.agent

    def _capture_jetson(self):
        try:
            frame = self._get_agent().capture_metric_rgbd(retries=2)
        except Exception:
            if self.agent is not None:
                self.agent.close()
                self.agent = None
            raise
        camera_to_world = np.asarray(frame["camera_to_world"], dtype=float)
        world_to_robot = pose_matrix(frame["robot_pose"])
        return {
            "image": frame["rgb"],
            "depth": frame["depth_m"],
            "intrinsics": frame["intrinsics"],
            "camera_to_robot": np.linalg.inv(world_to_robot) @ camera_to_world,
            "capture_world_frame": frame.get("world_frame", ""),
            "timestamp_ns": int(frame.get("timestamp_ns", 0)),
        }

    @staticmethod
    def _intrinsic_matrix(intrinsics):
        if isinstance(intrinsics, dict):
            return np.asarray([
                [float(intrinsics["fx"]), 0.0, float(intrinsics["cx"])],
                [0.0, float(intrinsics["fy"]), float(intrinsics["cy"])],
                [0.0, 0.0, 1.0],
            ])
        return np.asarray(intrinsics, dtype=float).reshape(3, 3)

    def _get_attribute_matcher(self):
        if self.attribute_matcher is None:
            from perception.textregion_siglip2 import TextRegionSigLIP2Matcher

            self.attribute_matcher = TextRegionSigLIP2Matcher(
                device=self.args.textregion_device,
                architecture=self.args.textregion_architecture,
                pretrained=self.args.textregion_checkpoint,
                dtype=self.args.textregion_dtype,
            )
        return self.attribute_matcher

    def _position_from_mask(self, frame, mask):
        depth = np.asarray(frame["depth"])
        if mask.shape != depth.shape:
            raise RuntimeError("semantic mask and aligned depth dimensions differ")
        if self.args.backend == "hyzx_snapshot":
            return robust_hyzx_position(
                mask, depth, frame["intrinsics"], frame["camera_to_robot"]
            )
        return robust_black_position(
            mask, depth, frame["intrinsics"], frame["camera_to_robot"]
        )

    def _evaluate_query(self, frame, query, detections_by_prompt, ocr_detections):
        """Fuse both discovery paths, apply four-field constraints and localize."""
        self.last_visualization_path = ""
        target_query = self.TargetQuery.build(
            target=query["target"],
            attributes=" ".join(query["attributes"]),
            text_constraints=query["text_constraints"],
            relations=query["relations"],
        )
        sam3_detections = detections_by_prompt.get(query["visual_prompt"], [])
        base = self.fuse_sam3_ocr(
            sam3_detections,
            ocr_detections,
            target=query["target"],
            visual_prompt=query["visual_prompt"],
            ocr_target=query["ocr_target"],
            ocr_aliases=query["ocr_aliases"],
            expected_instance=self.expected_instances.get(query["query_id"]),
            min_ocr_confidence=self.args.ocr_confidence,
            min_text_mask_coverage=self.args.min_ocr_mask_coverage,
        )
        structured = bool(
            target_query.attributes
            or target_query.text_constraints
            or target_query.relations
        )
        fusion = base
        if structured:
            fusion = self.apply_structured_query(
                base,
                query=target_query,
                image_bgr=frame["image"],
                sam3_detections=sam3_detections,
                ocr_detections=ocr_detections,
                attribute_matcher=(
                    self._get_attribute_matcher() if target_query.attributes else None
                ),
                reference_detections={
                    reference: detections_by_prompt.get(reference, [])
                    for reference in target_query.relation_references
                },
                min_ocr_confidence=self.args.ocr_confidence,
                min_text_mask_coverage=self.args.min_ocr_mask_coverage,
                require_single=True,
                depth_m=frame["depth"],
                intrinsics=self._intrinsic_matrix(frame["intrinsics"]),
                # Rigid-frame distances are invariant, so robot-local geometry
                # is sufficient for nearest/farthest structured relations.
                camera_to_world=np.asarray(frame["camera_to_robot"], dtype=float),
                robot_pose=[0.0] * 6,
                pose_estimator=self.pose_estimator,
            )

        if fusion.get("decision") != "STOP" or fusion.get("bbox_xyxy") is None:
            reason = str(fusion.get("reason") or "semantic target was not resolved")
            self._write_visualization(frame["image"], query["target"], error=reason)
            return {
                "success": False,
                "query_id": query["query_id"],
                "purpose": query["purpose"],
                "target": query["target"],
                "semantic_class": query["semantic_class"],
                "decision": fusion.get("decision", "RESELECT"),
                "reason": reason,
                "target_candidate_count": int(fusion.get("target_candidate_count", 0)),
                "timestamp_ns": int(frame.get("timestamp_ns", 0)),
                "visualization_path": self.last_visualization_path,
            }

        geometry_source = fusion.get("geometry_source")
        if geometry_source == "sam3":
            selected_index = fusion.get("selected_sam3_index")
            if not isinstance(selected_index, int) or not 0 <= selected_index < len(sam3_detections):
                raise RuntimeError("semantic fusion selected an invalid SAM3 candidate")
            segmentation = self.sam3.segment_from_detection(sam3_detections[selected_index])
        elif geometry_source == "ocr":
            selected_index = fusion.get("selected_ocr_index")
            if not isinstance(selected_index, int) or not 0 <= selected_index < len(ocr_detections):
                raise RuntimeError("semantic fusion selected an invalid OCR candidate")
            segmentation = self.ocr.segment_from_detection(
                ocr_detections[selected_index], frame["image"].shape
            )
        else:
            raise RuntimeError(f"unknown semantic geometry source: {geometry_source!r}")
        if not segmentation.get("success"):
            raise RuntimeError(str(segmentation.get("reason") or "segmentation failed"))

        mask = np.asarray(segmentation["mask"], dtype=bool)
        camera, robot, pixels = self._position_from_mask(frame, mask)
        if self.args.backend != "hyzx_snapshot":
            artifact_dir = Path(self.args.artifact_dir)
            artifact_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(artifact_dir / "latest_rgb.jpg"), frame["image"])
            cv2.imwrite(str(artifact_dir / "latest_mask.png"), mask.astype(np.uint8) * 255)
        instance_id = str(fusion.get("instance_id") or "").strip()
        if instance_id:
            self.expected_instances[query["query_id"]] = instance_id
        detection = {
            "mask": mask,
            "bbox_xyxy": fusion["bbox_xyxy"],
            "confidence": float(fusion.get("confidence", 0.0)),
        }
        result = {
            "success": True,
            "query_id": query["query_id"],
            "purpose": query["purpose"],
            "target": query["target"],
            "semantic_class": query["semantic_class"],
            "structured_query": target_query.to_dict(),
            "decision": "STOP",
            "verification_mode": fusion.get("verification_mode"),
            "geometry_source": geometry_source,
            "confidence_source": fusion.get("confidence_source"),
            "confidence": float(fusion.get("confidence", 0.0)),
            "sam3_confidence": fusion.get("sam3_confidence"),
            "ocr_confidence": fusion.get("ocr_confidence"),
            "ocr_evidence": fusion.get("ocr_evidence", []),
            "instance_id": fusion.get("instance_id"),
            "bbox_xyxy": [float(value) for value in fusion["bbox_xyxy"]],
            "valid_depth_pixels": int(pixels),
            "mask_pixels": int(np.count_nonzero(mask)),
            "valid_depth_ratio": float(pixels / max(np.count_nonzero(mask), 1)),
            "position_camera_optical_m": camera.tolist(),
            "position_robot_m": robot.tolist(),
            "robot_frame": self.args.robot_frame,
            "capture_world_frame": frame.get("capture_world_frame", ""),
            "timestamp_ns": int(frame.get("timestamp_ns", 0)),
            "visualization_path": self._write_visualization(
                frame["image"], query["target"], detection=detection
            ),
        }
        for key in ("rgb_path", "depth_path"):
            if key in frame:
                result[key] = frame[key]
        return result

    def detect(self, request):
        raw_queries = request.get("queries")
        if raw_queries is None:
            prompts = request.get("prompts")
            if prompts is None:
                prompts = [request.get("prompt") or self.args.prompt]
            raw_queries = [legacy_prompt_query(value) for value in prompts if str(value).strip()]
        queries = [normalize_detection_query(value) for value in raw_queries]
        if not queries:
            raise ValueError("structured target query list is empty")
        frame = (
            self._capture_hyzx()
            if self.args.backend == "hyzx_snapshot"
            else self._capture_jetson()
        )
        prompt_order = []
        for query in queries:
            specification = self.TargetQuery.build(
                target=query["target"], attributes=" ".join(query["attributes"]),
                text_constraints=query["text_constraints"], relations=query["relations"],
            )
            for prompt in (query["visual_prompt"], *specification.relation_references):
                if prompt and prompt not in prompt_order:
                    prompt_order.append(prompt)
        with open(self.lock_path, "w", encoding="utf-8") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            prompted = self.sam3.detect_many(frame["image"], prompt_order)
            ocr_detections = self.ocr.detect(frame["image"])
            detections_by_prompt = dict(zip(prompt_order, prompted))

        observations = []
        for query in queries:
            try:
                observations.append(
                    self._evaluate_query(frame, query, detections_by_prompt, ocr_detections)
                )
            except Exception as error:
                result = {
                    "success": False, "query_id": query["query_id"],
                    "purpose": query["purpose"], "target": query["target"],
                    "semantic_class": query["semantic_class"], "decision": "RESELECT",
                    "reason": str(error), "timestamp_ns": int(frame.get("timestamp_ns", 0)),
                }
                if self.last_visualization_path:
                    result["visualization_path"] = self.last_visualization_path
                observations.append(result)
        if len(queries) == 1:
            return observations[0]
        return {"success": any(item["success"] for item in observations),
                "timestamp_ns": int(frame.get("timestamp_ns", 0)),
                "observations": observations}

    def safe_detect(self, request):
        try:
            return self.detect(request)
        except Exception as error:
            result = {"success": False, "reason": str(error)}
            if self.last_visualization_path:
                result["visualization_path"] = self.last_visualization_path
            return result

    def close(self):
        if self.agent is not None:
            self.agent.close()
            self.agent = None


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serve-dir", required=True)
    parser.add_argument("--backend", choices=("hyzx_snapshot", "jetson_rpc"), required=True)
    parser.add_argument("--detector-root", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--ocr-detection-model", required=True)
    parser.add_argument("--ocr-recognition-model", required=True)
    parser.add_argument("--ocr-device", default="cpu")
    parser.add_argument("--ocr-version", default="PP-OCRv6")
    parser.add_argument("--ocr-confidence", type=float, default=0.5)
    parser.add_argument("--min-ocr-mask-coverage", type=float, default=0.10)
    parser.add_argument("--textregion-checkpoint", required=True)
    parser.add_argument("--textregion-device", default="cuda:0")
    parser.add_argument("--textregion-architecture", default="ViT-L-16-SigLIP2-256")
    parser.add_argument("--textregion-dtype", choices=("bf16", "fp32"), default="bf16")
    parser.add_argument("--prompt", default="white box")
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument("--gpu-lock", required=True)
    parser.add_argument("--cybereye-root")
    parser.add_argument("--capture-dir")
    parser.add_argument("--camera-to-robot")
    parser.add_argument("--black-root")
    parser.add_argument("--artifact-dir")
    parser.add_argument("--mqtt-env-file")
    parser.add_argument("--broker")
    parser.add_argument("--port", type=int, default=8883)
    parser.add_argument("--device-id", default="jetson003")
    parser.add_argument("--world-frame", default="odom")
    parser.add_argument("--robot-frame", default="base_link")
    return parser


def main():
    args = build_parser().parse_args()
    serve_dir = Path(args.serve_dir)
    engine = None
    try:
        engine = PersistentTargetEngine(args)
        serve_requests(serve_dir, engine.safe_detect)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        atomic_write_json(serve_dir / STARTUP_ERROR_FILE, {"reason": str(error)})
        return 2
    finally:
        if engine is not None:
            engine.close()


if __name__ == "__main__":
    raise SystemExit(main())
