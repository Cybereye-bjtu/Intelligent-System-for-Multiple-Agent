"""Detect a configured target in vehicle RGB-D and send a safe nav goal."""

import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import cv2
import rclpy
from aaa_search_interfaces.msg import TargetObservation
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .kettle_navigation_geometry import approach_pose, transform_point
from .observation_confirmation import ConfirmationHistory
from .persistent_detection_protocol import PersistentWorkerProcess
from .semantic_detection_contract import (
    legacy_prompt_query,
    normalize_detection_query,
    topology_detection_query,
)
from .semantic_target_config import load_topology_objects_file, parse_topology_objects


def capture_time_from_result(result):
    timestamp_ns = int(result.get("timestamp_ns", 0))
    if timestamp_ns <= 0:
        raise ValueError("detector result is missing the RGB-D capture timestamp")
    return Time(nanoseconds=timestamp_ns)


class KettleNavigator(Node):
    def __init__(self):
        super().__init__("kettle_navigator")
        defaults = {
            "goal_topic": "/hyzx001/goal_pose",
            "target_pose_topic": "/hyzx001/kettle_pose",
            "status_topic": "/hyzx001/kettle_navigation/status",
            "visualization_topic": "/hyzx001/target_detection/image",
            "map_frame": "site_map",
            "robot_frame": "hyzx001/base_footprint",
            "standoff_distance": 0.30,
            "minimum_target_distance": 0.25,
            "prompt": "red water bottle",
            "confidence_threshold": 0.50,
            "capture_timeout": 25.0,
            "auto_start": False,
            "worker_backend": "hyzx_snapshot",
            "cybereye_root": "/home/szhang/workspace/cybereye_target/cybereye_nl_target_pose",
            "sam3_runner": "/home/szhang/workspace/cybereye_target/.venv/bin/python",
            "sam3_checkpoint": "/data2/szhang/model/sam3/sam3.pt",
            "ocr_detection_model": "/data2/szhang/model/PP-OCRv6_medium_det",
            "ocr_recognition_model": "/data2/szhang/model/PP-OCRv6_medium_rec",
            "ocr_device": "cpu",
            "ocr_confidence_threshold": 0.50,
            "min_ocr_mask_coverage": 0.10,
            "textregion_checkpoint": "/data2/szhang/model/ViT-L-16-SigLIP2-256/open_clip_pytorch_model.bin",
            "textregion_device": "cuda:0",
            "textregion_architecture": "ViT-L-16-SigLIP2-256",
            "textregion_dtype": "bf16",
            "camera_to_robot": "/home/szhang/workspace/cybereye_target/cybereye_nl_target_pose/config/camera_to_base_footprint.json",
            "capture_dir": "/home/szhang/.local/share/aaa_ros2_multirobot_find_target/kettle_rgbd_capture",
            "black_root": "/data2/szhang/cybereye_nl_target_pose_black_vehicle",
            "black_artifact_dir": "/home/szhang/.local/share/aaa_ros2_multirobot_find_target/jetson003_target_capture",
            "mqtt_env_file": "/home/szhang/workspace/aaa_ros2_multirobot_find_target/.jetson003_mqtt.env",
            "mqtt_broker": "ca15b49bc8b442638f0cade1e45585ce.s1.eu.hivemq.cloud",
            "mqtt_port": 8883,
            "device_id": "jetson003",
            "capture_world_frame": "odom",
            "capture_robot_frame": "base_link",
            "persistent_worker": True,
            "worker_startup_timeout": 240.0,
            "gpu_index": 0,
            "continuous_detection": False,
            "continuous_enabled_topic": "/search/exploration_enabled",
            "mission_id_topic": "/search/mission_id",
            "target_prompt_topic": "/search/target_prompt",
            "target_query_topic": "/search/target_query",
            "candidate_topic": "/search/target_candidate",
            "observation_topic": "/search/target_observation/hyzx001",
            "semantic_observation_topic": "/semantic_topology/observation/hyzx001",
            "confirmation_count": 1,
            "confirmation_distance_m": 0.30,
            "detection_interval": 3.0,
            "continuous_require_mission_id": True,
            "semantic_mapping_mode": False,
            "topology_objects_file": "",
            "semantic_targets_json": json.dumps({"semantic_targets": [
                {"target": "box", "attributes": ["white"],
                 "text": "school", "relations": []},
                {"target": "box", "attributes": ["white"],
                 "text": "bank", "relations": []},
            ]}),
            "semantic_targets_topic": "/semantic_topology/targets_config",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.goal_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self.target_publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter("target_pose_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.visualization_publisher = self.create_publisher(
            Image, str(self.get_parameter("visualization_topic").value), 2
        )
        self.latest_visualization = None
        self.visualization_timer = self.create_timer(
            1.0, self._republish_visualization
        )
        self.candidate_publisher = self.create_publisher(
            String, str(self.get_parameter("candidate_topic").value), 10
        )
        self.observation_publisher = self.create_publisher(
            TargetObservation, str(self.get_parameter("observation_topic").value), 10
        )
        self.semantic_observation_publisher = self.create_publisher(
            TargetObservation,
            str(self.get_parameter("semantic_observation_topic").value), 10,
        )
        # Inference may finish well after acquisition.  Retain enough TF history
        # to transform the result with the pose that belonged to the RGB-D frame.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=300.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.busy = False
        self.exploration_enabled = False
        self.target_latched = False
        self.mission_id = ""
        self.target_prompt = str(self.get_parameter("prompt").value).strip()
        self.mission_query = None
        self.pending_mission_query = None
        topology_objects_file = str(
            self.get_parameter("topology_objects_file").value
        ).strip()
        if topology_objects_file:
            self.topology_objects = load_topology_objects_file(topology_objects_file)
            self.get_logger().info(
                f"loaded {len(self.topology_objects)} topology objects "
                f"from {topology_objects_file}"
            )
        else:
            # Compatibility fallback for deployments that still inject JSON as
            # a ROS parameter instead of using the persistent object dictionary.
            self.topology_objects = parse_topology_objects(
                str(self.get_parameter("semantic_targets_json").value)
            )
        self.confirmations = ConfirmationHistory()
        self.last_detection_started = 0.0
        self.worker = None
        if bool(self.get_parameter("persistent_worker").value):
            gpu_index = int(self.get_parameter("gpu_index").value)
            package_python_root = str(Path(__file__).resolve().parents[1])
            inherited_pythonpath = os.environ.get("PYTHONPATH", "")
            worker_pythonpath = package_python_root
            if inherited_pythonpath:
                worker_pythonpath += os.pathsep + inherited_pythonpath
            self.worker = PersistentWorkerProcess(
                self._persistent_worker_command(),
                environment={
                    "CUDA_VISIBLE_DEVICES": str(gpu_index),
                    "PYTHONPATH": worker_pythonpath,
                },
                startup_timeout=float(
                    self.get_parameter("worker_startup_timeout").value
                ),
            )
        self.service = self.create_service(Trigger, "~/detect_and_go", self.trigger)
        self.create_subscription(
            Bool,
            str(self.get_parameter("continuous_enabled_topic").value),
            self._on_exploration_enabled,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_id_topic").value),
            self._on_mission_id,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_prompt_topic").value),
            self._on_target_prompt,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("target_query_topic").value),
            self._on_target_query,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("semantic_targets_topic").value),
            self._on_semantic_targets,
            10,
        )
        self.continuous_timer = self.create_timer(0.25, self._continuous_tick)
        if bool(self.get_parameter("auto_start").value):
            self.auto_timer = self.create_timer(3.0, self._auto_start)
        mode = "persistent model warming in background" if self.worker else "one-shot model"
        self._status(f"ready ({mode}); call ~/detect_and_go to detect and navigate")

    def _status(self, text):
        self.status_publisher.publish(String(data=str(text)))
        self.get_logger().info(str(text))

    def _auto_start(self):
        self.auto_timer.cancel()
        self._start_detection(navigate=True, source="auto")

    def _on_exploration_enabled(self, message):
        enabled = bool(message.data)
        if enabled and not self.exploration_enabled:
            self.target_latched = False
            self.confirmations.clear()
            self.last_detection_started = 0.0
        self.exploration_enabled = enabled

    def _clear_confirmations(self, stream=None):
        self.confirmations.clear(stream)

    def _on_mission_id(self, message):
        mission_id = str(message.data).strip()
        if mission_id != self.mission_id:
            self.mission_id = mission_id
            self.mission_query = None
            self.target_latched = False
            self._clear_confirmations("mission")
            self.last_detection_started = 0.0
            if (
                self.pending_mission_query is not None
                and self.pending_mission_query[0] == mission_id
            ):
                _, query = self.pending_mission_query
                self.pending_mission_query = None
                self._accept_mission_query(query)

    def _on_target_prompt(self, message):
        prompt = str(message.data).strip()
        if prompt and prompt != self.target_prompt:
            self.target_prompt = prompt
            self.mission_query = legacy_prompt_query(prompt)
            self.target_latched = False
            self._clear_confirmations("mission")
            self.last_detection_started = 0.0
            self._status(f"continuous detector prompt updated to {prompt!r}")

    def _on_target_query(self, message):
        try:
            document = json.loads(message.data)
            query_mission_id = str(document.pop("mission_id", "")).strip()
            query = normalize_detection_query(document, default_purpose="mission")
            if query_mission_id and not self.mission_id:
                self.pending_mission_query = (query_mission_id, query)
                self.get_logger().info(
                    "held structured query until its mission id is received"
                )
                return
            if query_mission_id and query_mission_id != self.mission_id:
                self.get_logger().warning("ignored structured query for a stale mission")
                return
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"rejected structured mission query: {error}")
            return
        self._accept_mission_query(query)

    def _accept_mission_query(self, query):
        self.mission_query = query
        self.target_prompt = query["semantic_class"]
        self.target_latched = False
        self._clear_confirmations("mission")
        self.last_detection_started = 0.0
        self._status(
            f"structured mission query ready: target={query['target']!r}, "
            f"text={query['text_constraints']!r}"
        )

    def _on_semantic_targets(self, message):
        try:
            targets = parse_topology_objects(message.data, require_all_fields=True)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"rejected topology object configuration: {error}")
            return
        self.topology_objects = targets
        self._clear_confirmations("topology")
        self.last_detection_started = 0.0
        self._status(
            "topology objects updated: "
            + ", ".join(item["prompt"] for item in targets)
        )

    def _continuous_tick(self):
        if not bool(self.get_parameter("continuous_detection").value):
            return
        mapping_mode = bool(self.get_parameter("semantic_mapping_mode").value)
        if not self.exploration_enabled or self.busy or (self.target_latched and not mapping_mode):
            return
        if (
            bool(self.get_parameter("continuous_require_mission_id").value)
            and not self.mission_id
        ):
            return
        interval = max(0.25, float(self.get_parameter("detection_interval").value))
        now = time.monotonic()
        if now - self.last_detection_started < interval:
            return
        self.last_detection_started = now
        self._start_detection(navigate=False, source="semantic_mapping" if mapping_mode else "continuous")

    def trigger(self, request, response):
        if self.busy:
            response.success = False
            response.message = "kettle detection is already running"
            return response
        self._start_detection(navigate=True, source="service")
        response.success = True
        response.message = "RGB-D capture and structured SAM3+OCR detection started"
        return response

    def _start_detection(self, *, navigate, source):
        self.busy = True
        threading.Thread(
            target=self._detect_and_publish,
            kwargs={"navigate": bool(navigate), "source": str(source)},
            daemon=True,
        ).start()

    def _persistent_worker_command(self):
        backend = str(self.get_parameter("worker_backend").value)
        gpu_index = int(self.get_parameter("gpu_index").value)
        command = [
            str(self.get_parameter("sam3_runner").value),
            "-m",
            "aaa_navigation.persistent_target_worker",
            "--backend", backend,
            "--detector-root", str(self.get_parameter("cybereye_root").value),
            "--checkpoint", str(self.get_parameter("sam3_checkpoint").value),
            "--confidence", str(self.get_parameter("confidence_threshold").value),
            "--ocr-detection-model", str(self.get_parameter("ocr_detection_model").value),
            "--ocr-recognition-model", str(self.get_parameter("ocr_recognition_model").value),
            "--ocr-device", str(self.get_parameter("ocr_device").value),
            "--ocr-confidence", str(self.get_parameter("ocr_confidence_threshold").value),
            "--min-ocr-mask-coverage", str(self.get_parameter("min_ocr_mask_coverage").value),
            "--textregion-checkpoint", str(self.get_parameter("textregion_checkpoint").value),
            "--textregion-device", str(self.get_parameter("textregion_device").value),
            "--textregion-architecture", str(self.get_parameter("textregion_architecture").value),
            "--textregion-dtype", str(self.get_parameter("textregion_dtype").value),
            "--prompt", str(self.get_parameter("prompt").value),
            "--timeout", str(self.get_parameter("capture_timeout").value),
            "--gpu-lock", f"/tmp/aaa_sam3_gpu_{os.getuid()}_{gpu_index}.lock",
            "--robot-frame", str(self.get_parameter("capture_robot_frame").value),
        ]
        if backend == "hyzx_snapshot":
            return command + [
                "--cybereye-root", str(self.get_parameter("cybereye_root").value),
                "--capture-dir", str(self.get_parameter("capture_dir").value),
                "--camera-to-robot", str(self.get_parameter("camera_to_robot").value),
            ]
        if backend == "jetson_rpc":
            return command + [
                "--black-root", str(self.get_parameter("black_root").value),
                "--artifact-dir", str(self.get_parameter("black_artifact_dir").value),
                "--mqtt-env-file", str(self.get_parameter("mqtt_env_file").value),
                "--broker", str(self.get_parameter("mqtt_broker").value),
                "--port", str(self.get_parameter("mqtt_port").value),
                "--device-id", str(self.get_parameter("device_id").value),
                "--world-frame", str(self.get_parameter("capture_world_frame").value),
            ]
        raise ValueError(f"unsupported worker_backend: {backend}")

    def _worker_command(self, result_path):
        root = str(self.get_parameter("cybereye_root").value)
        backend = str(self.get_parameter("worker_backend").value)
        common = [
            str(self.get_parameter("sam3_runner").value), "-m",
        ]
        if backend == "jetson_rpc":
            return common + [
                "aaa_navigation.black_target_detection_worker",
                "--result", result_path,
                "--artifact-dir", str(self.get_parameter("black_artifact_dir").value),
                "--black-root", str(self.get_parameter("black_root").value),
                "--detector-root", root,
                "--checkpoint", str(self.get_parameter("sam3_checkpoint").value),
                "--mqtt-env-file", str(self.get_parameter("mqtt_env_file").value),
                "--broker", str(self.get_parameter("mqtt_broker").value),
                "--port", str(self.get_parameter("mqtt_port").value),
                "--device-id", str(self.get_parameter("device_id").value),
                "--world-frame", str(self.get_parameter("capture_world_frame").value),
                "--robot-frame", str(self.get_parameter("capture_robot_frame").value),
                "--prompt", str(self.get_parameter("prompt").value),
                "--confidence", str(self.get_parameter("confidence_threshold").value),
                "--timeout", str(self.get_parameter("capture_timeout").value),
            ]
        if backend != "hyzx_snapshot":
            raise ValueError(f"unsupported worker_backend: {backend}")
        return [
            *common,
            "aaa_navigation.kettle_detection_worker", "--result", result_path,
            "--capture-dir", str(self.get_parameter("capture_dir").value),
            "--cybereye-root", root,
            "--checkpoint", str(self.get_parameter("sam3_checkpoint").value),
            "--camera-to-robot", str(self.get_parameter("camera_to_robot").value),
            "--prompt", str(self.get_parameter("prompt").value),
            "--confidence", str(self.get_parameter("confidence_threshold").value),
            "--timeout", str(self.get_parameter("capture_timeout").value),
        ]

    def _run_detection(self, query):
        timeout = float(self.get_parameter("capture_timeout").value) + 150.0
        if self.worker is not None:
            return self.worker.request({"queries": [query]}, timeout=timeout)
        raise RuntimeError(
            "structured SAM3+OCR detection requires persistent_worker=true"
        )

    def _run_semantic_batch(self):
        queries = [topology_detection_query(item) for item in self.topology_objects]
        # Preserve the original find-target mission while semantic mapping is
        # enabled. It shares the same keyframe but is not added to the topology
        # unless it is also in the persistent topology object dictionary.
        if self.mission_query is not None:
            queries.append(self.mission_query)
        if not queries:
            raise ValueError("topology object configuration is empty")
        if self.worker is None:
            # One-shot workers cannot retain a shared frame. Refuse instead of
            # silently violating the semantic-keyframe contract.
            raise RuntimeError("semantic mapping requires persistent_worker=true")
        timeout = float(self.get_parameter("capture_timeout").value) + 150.0 * len(queries)
        return self.worker.request({"queries": queries}, timeout=timeout)

    def _publish_visualization(self, result):
        path = str(result.get("visualization_path", ""))
        if not path:
            return
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning(f"detection visualization is unreadable: {path}")
            return
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.robot_frame
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = int(image.shape[1] * 3)
        message.data = image.tobytes()
        self.latest_visualization = message
        self.visualization_publisher.publish(message)

    def _republish_visualization(self):
        if self.latest_visualization is not None:
            self.visualization_publisher.publish(self.latest_visualization)

    def _record_confirmation(
        self, result, local, prompt, publisher=None, *, stream="mission"
    ):
        capture_time = capture_time_from_result(result)
        current = {
            "position": [float(value) for value in local],
            "confidence": float(result["confidence"]),
            "depth_ratio": float(result.get("valid_depth_ratio", 0.0)),
            "timestamp_ns": capture_time.nanoseconds,
        }
        required = max(1, int(self.get_parameter("confirmation_count").value))
        confirmations = self.confirmations.add(
            stream,
            prompt,
            current,
            maximum_count=required,
            distance_limit=float(
                self.get_parameter("confirmation_distance_m").value
            )
        )
        if len(confirmations) < required:
            return False

        positions = [item["position"] for item in confirmations]
        mean = [sum(point[index] for point in positions) / required for index in range(3)]
        variances = [
            max(
                0.0025,
                sum((point[index] - mean[index]) ** 2 for point in positions)
                / max(required - 1, 1),
            )
            for index in range(3)
        ]
        observation = TargetObservation()
        observation.header.stamp = Time(
            nanoseconds=int(confirmations[-1]["timestamp_ns"])
        ).to_msg()
        observation.header.frame_id = self.robot_frame
        observation.mission_id = self.mission_id
        observation.robot_id = self.get_namespace().strip("/")
        observation.target_label = prompt
        observation.instance_id = f"{self.mission_id}:{observation.robot_id}:{prompt}"
        observation.position.x, observation.position.y, observation.position.z = mean
        observation.position_covariance = [
            variances[0], 0.0, 0.0,
            0.0, variances[1], 0.0,
            0.0, 0.0, variances[2],
        ]
        observation.semantic_confidence = min(
            item["confidence"] for item in confirmations
        )
        observation.valid_depth_ratio = min(
            item["depth_ratio"] for item in confirmations
        )
        observation.observation_count = required
        (publisher or self.observation_publisher).publish(observation)
        return True

    def _detect_and_publish(self, *, navigate, source):
        try:
            prompt = self.target_prompt
            if source == "semantic_mapping":
                targets = [item["semantic_class"] for item in self.topology_objects]
                self._status(f"capturing one synchronized RGB-D keyframe for {targets!r}")
                batch = self._run_semantic_batch()
                for result in batch.get("observations", []):
                    if result.get("success"):
                        # Topology observations have a separate topic so the
                        # target-found bridge cannot stop exploration merely
                        # because a mapped school/bank box was seen.
                        if result.get("purpose") == "topology":
                            self._record_confirmation(
                                result, result["position_robot_m"],
                                result["semantic_class"],
                                publisher=self.semantic_observation_publisher,
                                stream="topology")
                        if result.get("purpose") == "mission":
                            self._record_confirmation(
                                result, result["position_robot_m"],
                                result["semantic_class"])
                # The worker annotates both hits and misses. Publish the final
                # query from every keyframe, then the existing 1 Hz timer keeps
                # the monitoring topic alive until the next keyframe arrives.
                observations = batch.get("observations", [])
                if observations:
                    self._publish_visualization(observations[-1])
                found = sum(bool(item.get("success")) for item in batch.get("observations", []))
                self._status(
                    f"semantic keyframe complete: {found}/"
                    f"{len(batch.get('observations', []))} queries detected"
                )
                return
            query = self.mission_query or legacy_prompt_query(prompt)
            self._status(
                f"capturing synchronized RGB-D and running SAM3+OCR for {prompt!r}"
            )
            result = self._run_detection(query)
            self._publish_visualization(result)
            if not result.get("success"):
                raise RuntimeError(result.get("reason", "unknown detector failure"))
            local = result["position_robot_m"]
            capture_time = capture_time_from_result(result)
            distance = math.hypot(float(local[0]), float(local[1]))
            minimum = float(self.get_parameter("minimum_target_distance").value)
            if distance < minimum:
                raise RuntimeError(
                    f"target distance {distance:.3f} m is below safety minimum {minimum:.3f} m"
                )
            if not navigate:
                candidate = {
                    "mission_id": self.mission_id,
                    "robot_frame": self.robot_frame,
                    "prompt": prompt,
                    "confidence": float(result["confidence"]),
                    "bbox_xyxy": result.get("bbox_xyxy", []),
                    "position_robot_m": [float(value) for value in local],
                    "valid_depth_pixels": int(result.get("valid_depth_pixels", 0)),
                    "valid_depth_ratio": float(result.get("valid_depth_ratio", 0.0)),
                    "timestamp_ns": int(result.get("timestamp_ns", 0)),
                }
                self.candidate_publisher.publish(
                    String(data=json.dumps(candidate, separators=(",", ":")))
                )
                confirmed = self._record_confirmation(result, local, prompt)
                self.target_latched = confirmed
                if confirmed:
                    self._status(
                        f"target confirmed for mission {self.mission_id}: "
                        f"{prompt!r} confidence={float(result['confidence']):.3f}; "
                        "published TargetObservation"
                    )
                else:
                    self._status(
                        f"candidate detected for mission {self.mission_id}: "
                        f"{prompt!r} confidence={float(result['confidence']):.3f}; "
                        f"confirmation {self.confirmations.count('mission', prompt)}/"
                        f"{int(self.get_parameter('confirmation_count').value)}"
                    )
                return
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                capture_time,
                timeout=Duration(seconds=3.0),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            target = transform_point(
                local,
                [translation.x, translation.y, translation.z],
                [rotation.x, rotation.y, rotation.z, rotation.w],
            )
            current_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=3.0),
            )
            current_translation = current_transform.transform.translation
            robot = [current_translation.x, current_translation.y]
            goal_x, goal_y, yaw, target_distance = approach_pose(
                robot, target[:2], float(self.get_parameter("standoff_distance").value)
            )
            capture_stamp = capture_time.to_msg()
            goal_stamp = self.get_clock().now().to_msg()
            target_message = PoseStamped()
            target_message.header.frame_id = self.map_frame
            target_message.header.stamp = capture_stamp
            target_message.pose.position.x = float(target[0])
            target_message.pose.position.y = float(target[1])
            target_message.pose.position.z = float(target[2])
            target_message.pose.orientation.w = 1.0
            self.target_publisher.publish(target_message)
            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = goal_stamp
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.goal_publisher.publish(goal)
            self._status(
                "%s=(%.3f, %.3f, %.3f) %s, confidence=%.3f; "
                "navigation goal=(%.3f, %.3f), observed range=%.3f m"
                % (
                    prompt, target[0], target[1], target[2], self.map_frame,
                    float(result["confidence"]), goal_x, goal_y, target_distance,
                )
            )
        except (RuntimeError, OSError, ValueError, subprocess.SubprocessError, TransformException) as error:
            if source == "continuous":
                self._clear_confirmations("mission")
            level = (
                self.get_logger().warning
                if source == "continuous"
                else self.get_logger().error
            )
            level(f"target detection failed ({source}): {error}")
            self.status_publisher.publish(
                String(
                    data=(
                        f"scan-miss: {error}"
                        if source == "continuous"
                        else f"failed: {error}"
                    )
                )
            )
        finally:
            self.busy = False

    def destroy_node(self):
        if self.worker is not None:
            self.worker.close()
            self.worker = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KettleNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
