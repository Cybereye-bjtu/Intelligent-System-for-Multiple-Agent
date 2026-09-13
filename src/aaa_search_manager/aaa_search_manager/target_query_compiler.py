"""Compile each accepted natural-language mission into one four-field query."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

from .query_compiler_contract import compiler_command


class TargetQueryCompilerNode(Node):
    def __init__(self):
        super().__init__("target_query_compiler")
        defaults = {
            "accepted_query_topic": "/search/accepted_mission_query",
            "target_query_topic": "/search/target_query",
            "status_topic": "/search/target_query/status",
            "reset_topic": "/search/reset",
            "runner": "/home/szhang/workspace/cybereye_target/.venv/bin/python",
            "module_root": "/home/szhang/workspace/cybereye_target/cybereye_nl_target_pose",
            "model_path": "/data2/szhang/model/Qwen3.5-2B",
            "adapter_path": "/data2/szhang/model/qwen35-2b-cybereye-extractor-lora-v2",
            "mode": "hybrid",
            "max_new_tokens": 512,
            "timeout_sec": 300.0,
            "gpu_index": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.query_publisher = self.create_publisher(
            String, str(self.get_parameter("target_query_topic").value), qos
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("accepted_query_topic").value),
            self._on_query,
            qos,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("reset_topic").value),
            self._on_reset,
            10,
        )
        self.generation = 0

    def _status(self, value):
        self.status_publisher.publish(String(data=str(value)))
        self.get_logger().info(str(value))

    def _on_reset(self, _message):
        # Invalidate an in-flight compile so an old mission cannot publish
        # after MissionManager has reset the search state.
        self.generation += 1
        self._status("target query compiler reset")

    def _on_query(self, message):
        try:
            value = json.loads(message.data)
            mission_id = str(value.get("mission_id") or "").strip()
            source = str(value.get("text") or "").strip()
            if not mission_id or not source:
                raise ValueError("accepted mission query requires mission_id and text")
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.get_logger().error(f"invalid accepted mission query: {error}")
            return
        self.generation += 1
        generation = self.generation
        self._status(f"compiling natural-language target for mission {mission_id}")
        threading.Thread(
            target=self._compile,
            args=(generation, mission_id, source),
            daemon=True,
        ).start()

    def _compile(self, generation, mission_id, source):
        runner = Path(str(self.get_parameter("runner").value))
        model = Path(str(self.get_parameter("model_path").value))
        adapter = Path(str(self.get_parameter("adapter_path").value))
        try:
            if not runner.is_file() or not model.is_dir() or not adapter.is_dir():
                raise FileNotFoundError("query runner/model/adapter path is missing")
            command = compiler_command(
                runner,
                text=source,
                model=model,
                adapter=adapter,
                mode=str(self.get_parameter("mode").value),
                max_new_tokens=int(self.get_parameter("max_new_tokens").value),
            )
            environment = os.environ.copy()
            module_root = str(self.get_parameter("module_root").value)
            inherited_pythonpath = environment.get("PYTHONPATH", "")
            environment["PYTHONPATH"] = (
                module_root if not inherited_pythonpath
                else module_root + os.pathsep + inherited_pythonpath
            )
            environment["CUDA_VISIBLE_DEVICES"] = str(
                int(self.get_parameter("gpu_index").value)
            )
            process = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=float(self.get_parameter("timeout_sec").value),
                env=environment,
            )
            if process.returncode != 0:
                detail = process.stderr.strip() or process.stdout.strip()
                raise RuntimeError(
                    f"query compiler exited {process.returncode}: {detail[-1200:]}"
                )
            query = json.loads(process.stdout)
            if generation != self.generation:
                return
            query.update({
                "mission_id": mission_id,
                "query_id": f"mission:{mission_id}",
                "purpose": "mission",
                "semantic_class": str(query.get("target") or source),
                "source_text": source,
            })
            self.query_publisher.publish(
                String(data=json.dumps(query, ensure_ascii=False, separators=(",", ":")))
            )
            self._status(
                f"structured target ready for mission {mission_id}: {query['target']!r}"
            )
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            if generation == self.generation:
                self.get_logger().error(
                    f"natural-language target compilation failed: {error}"
                )
                self.status_publisher.publish(String(data=f"failed: {error}"))


def main(args=None):
    rclpy.init(args=args)
    node = TargetQueryCompilerNode()
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
