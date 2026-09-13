"""ROS-independent semantic topology graph rules.

The module intentionally contains no planner-derived connectivity: an edge is
created only by :meth:`RobotGraph.update_pose` after a robot has visited both
endpoints during one valid trajectory segment.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from typing import Callable, Dict, List, Optional, Tuple


Point = Tuple[float, float]


def public_position(point: Optional[Point]) -> Optional[dict]:
    if point is None:
        return None
    return {"x": round(point[0], 4), "y": round(point[1], 4), "z": 0.0}


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_delta(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def label_slug(label: str) -> str:
    value = re.sub(r"[^a-z0-9]+", "-", label.strip().lower()).strip("-")
    return value or "object"


@dataclass
class NodeRecord:
    id: str
    label: str
    position: Point
    confidence: float
    weight: float
    observation_count: int = 1
    observation_positions: List[Point] = field(default_factory=list)
    last_observation_time: float = -math.inf
    confirmed: bool = False
    navigation_position: Optional[Point] = None

    def public(self, public_id: Optional[str] = None) -> dict:
        return {
            "id": public_id or self.id,
            "semantic_class": self.label,
            "position": public_position(self.position),
            "navigation_position": public_position(self.navigation_position),
            "confidence": round(self.confidence, 4),
            "observation_count": self.observation_count,
        }


@dataclass
class EdgeRecord:
    source: str
    target: str
    distance: float
    traversal_count: int = 1

    def public(self, id_mapping: Optional[Dict[str, str]] = None) -> dict:
        mapping = id_mapping or {}
        return {
            "source": mapping.get(self.source, self.source),
            "target": mapping.get(self.target, self.target),
            "distance": round(self.distance, 4),
            "traversed": True,
            "traversal_count": self.traversal_count,
        }


class RobotGraph:
    def __init__(self, robot_id: str, *, detection_confidence=0.50,
                 merge_distance=0.20, min_observations=1,
                 observation_interval=0.5, min_observation_baseline=0.15,
                 max_weight=10.0, stand_off_distance=0.30,
                 visit_radius=0.30, leave_radius=0.45,
                 sample_distance=0.04, sample_yaw_deg=5.0,
                 localization_jump_limit=0.20, minimum_path_length=0.30):
        self.robot_id = robot_id
        self.detection_confidence = float(detection_confidence)
        self.merge_distance = float(merge_distance)
        self.min_observations = int(min_observations)
        self.observation_interval = float(observation_interval)
        self.min_observation_baseline = float(min_observation_baseline)
        self.max_weight = float(max_weight)
        self.stand_off_distance = float(stand_off_distance)
        self.visit_radius = float(visit_radius)
        self.leave_radius = float(leave_radius)
        self.sample_distance = float(sample_distance)
        self.sample_yaw = math.radians(float(sample_yaw_deg))
        self.localization_jump_limit = float(localization_jump_limit)
        self.minimum_path_length = float(minimum_path_length)
        self.nodes: Dict[str, NodeRecord] = {}
        self.edges: Dict[Tuple[str, str], EdgeRecord] = {}
        self._label_counts: Dict[str, int] = {}
        self.active_node: Optional[str] = None
        self.last_visited_node: Optional[str] = None
        self.trajectory: List[Tuple[float, float, float]] = []
        self.segment_valid = True

    def observe(self, label: str, position: Point, confidence: float,
                robot_position: Point, stamp: float,
                navigation_is_free: Callable[[Point], bool]) -> Optional[NodeRecord]:
        values = (*position, *robot_position, confidence, stamp)
        if not label.strip() or not all(math.isfinite(float(v)) for v in values):
            return None
        if confidence < self.detection_confidence:
            return None
        candidates = [n for n in self.nodes.values()
                      if n.label == label and distance(n.position, position) < self.merge_distance]
        node = min(candidates, key=lambda n: distance(n.position, position)) if candidates else None
        if node is None:
            slug = label_slug(label)
            count = self._label_counts.get(slug, 0) + 1
            self._label_counts[slug] = count
            node = NodeRecord(
                id=f"{slug}-{count:03d}", label=label, position=position,
                confidence=confidence, weight=min(confidence, self.max_weight),
                observation_positions=[robot_position], last_observation_time=stamp,
            )
            self.nodes[node.id] = node
        else:
            # High-rate duplicates are not effective observations and must not
            # move a node or increase its confirmation count.
            if stamp - node.last_observation_time < self.observation_interval:
                return node
            old_weight = min(node.weight, self.max_weight)
            denominator = old_weight + confidence
            node.position = (
                (node.position[0] * old_weight + position[0] * confidence) / denominator,
                (node.position[1] * old_weight + position[1] * confidence) / denominator,
            )
            node.weight = min(denominator, self.max_weight)
            node.confidence = (
                (node.confidence * (node.observation_count) + confidence)
                / (node.observation_count + 1)
            )
            node.observation_count += 1
            node.observation_positions.append(robot_position)
            node.last_observation_time = stamp

        has_baseline = self.min_observations <= 1 or any(
            distance(a, b) >= self.min_observation_baseline
            for i, a in enumerate(node.observation_positions)
            for b in node.observation_positions[i + 1:]
        )
        if not node.confirmed and node.observation_count >= self.min_observations and has_baseline:
            dx = robot_position[0] - node.position[0]
            dy = robot_position[1] - node.position[1]
            norm = math.hypot(dx, dy)
            if norm > 1e-6:
                candidate = (
                    node.position[0] + self.stand_off_distance * dx / norm,
                    node.position[1] + self.stand_off_distance * dy / norm,
                )
                node.confirmed = True
                node.navigation_position = candidate if navigation_is_free(candidate) else None
        return node

    def invalidate_segment(self) -> None:
        self.segment_valid = False
        self.trajectory.clear()

    def update_pose(self, x: float, y: float, yaw: float, *, localization_valid=True,
                    emergency_stop=False) -> Optional[EdgeRecord]:
        pose = (float(x), float(y), float(yaw))
        if not all(math.isfinite(v) for v in pose) or not localization_valid or emergency_stop:
            self.invalidate_segment()
            return None
        if self.trajectory:
            previous = self.trajectory[-1]
            step = distance(pose[:2], previous[:2])
            if step > self.localization_jump_limit:
                self.invalidate_segment()
            elif step >= self.sample_distance or angle_delta(pose[2], previous[2]) >= self.sample_yaw:
                self.trajectory.append(pose)

        eligible = [n for n in self.nodes.values()
                    if n.confirmed and n.navigation_position is not None]
        nearest = min(eligible, key=lambda n: distance(pose[:2], n.navigation_position), default=None)
        entered = nearest if nearest and distance(pose[:2], nearest.navigation_position) < self.visit_radius else None

        if self.active_node is not None:
            current = self.nodes[self.active_node]
            if distance(pose[:2], current.navigation_position) <= self.leave_radius:
                return None
            self.active_node = None
            if not self.trajectory:
                self.trajectory = [pose]
            return None

        if entered is None:
            if self.last_visited_node is not None and not self.trajectory:
                self.trajectory = [pose]
            return None

        self.active_node = entered.id
        if self.last_visited_node is None:
            self.last_visited_node = entered.id
            self.trajectory = [pose]
            self.segment_valid = True
            return None
        if entered.id == self.last_visited_node:
            self.trajectory = [pose]
            self.segment_valid = True
            return None

        path_length = sum(distance(a[:2], b[:2]) for a, b in zip(self.trajectory, self.trajectory[1:]))
        edge = None
        if self.segment_valid and path_length >= self.minimum_path_length:
            key = tuple(sorted((self.last_visited_node, entered.id)))
            edge = self.edges.get(key)
            if edge is None:
                edge = EdgeRecord(key[0], key[1], path_length)
                self.edges[key] = edge
            else:
                edge.distance = ((edge.distance * edge.traversal_count) + path_length) / (edge.traversal_count + 1)
                edge.traversal_count += 1
        self.last_visited_node = entered.id
        self.trajectory = [pose]
        self.segment_valid = True
        return edge

    def public(self, *, confirmed_only=False) -> dict:
        selected = [n for n in self.nodes.values() if n.confirmed or not confirmed_only]
        id_mapping = {node.id: f"N{index}" for index, node in enumerate(selected, 1)}
        nodes = [node.public(id_mapping[node.id]) for node in selected]
        allowed = set(id_mapping)
        edges = [e.public(id_mapping) for e in self.edges.values()
                 if e.source in allowed and e.target in allowed]
        return {"robot": self.robot_id, "nodes": nodes, "edges": edges}


class GraphFusion:
    """Incremental fusion which never reassigns an existing source mapping."""
    def __init__(self, merge_distance=0.20):
        self.merge_distance = float(merge_distance)
        self.nodes: Dict[str, dict] = {}
        self.edges: Dict[Tuple[str, str], dict] = {}
        self.mapping: Dict[str, str] = {}
        self._next_node_number = 1

    def update(self, graphs: Dict[str, RobotGraph]) -> dict:
        # First allocate mappings. Existing mappings are immutable; this makes
        # IDs stable even when later pose fusion moves a centroid.
        for robot_id, graph in graphs.items():
            for node in graph.nodes.values():
                if not node.confirmed:
                    continue
                source_id = f"{robot_id}-{node.id}"
                global_id = self.mapping.get(source_id)
                if global_id is None:
                    candidates = [value for value in self.nodes.values()
                                  if value["label"] == node.label
                                  and distance(tuple(value["position"]), node.position) < self.merge_distance]
                    if candidates:
                        target = min(candidates, key=lambda v: distance(tuple(v["position"]), node.position))
                        global_id = target["id"]
                    else:
                        global_id = f"N{self._next_node_number}"
                        self._next_node_number += 1
                        self.nodes[global_id] = {
                            "id": global_id, "label": node.label,
                            "position": list(node.position),
                            "navigation_position": list(node.navigation_position) if node.navigation_position else None,
                            "confidence": node.confidence, "observation_count": 0,
                        }
                    self.mapping[source_id] = global_id
        # Recompute statistics from source graphs, so repeated publication is
        # idempotent and never double-counts an unchanged observation.
        aggregates = {global_id: {**value, "position": [0.0, 0.0],
                                  "confidence": 0.0, "observation_count": 0}
                      for global_id, value in self.nodes.items()}
        for robot_id, graph in graphs.items():
            for node in graph.nodes.values():
                if not node.confirmed:
                    continue
                target = aggregates[self.mapping[f"{robot_id}-{node.id}"]]
                count = node.observation_count
                target["position"][0] += node.position[0] * count
                target["position"][1] += node.position[1] * count
                target["confidence"] += node.confidence * count
                target["observation_count"] += count
                if node.navigation_position is not None:
                    target["navigation_position"] = list(node.navigation_position)
        for global_id, target in aggregates.items():
            count = target["observation_count"]
            if count:
                target["position"] = [value / count for value in target["position"]]
                target["confidence"] /= count
            self.nodes[global_id] = target

        self.edges = {}
        for robot_id, graph in graphs.items():
            for edge in graph.edges.values():
                a = self.mapping.get(f"{robot_id}-{edge.source}")
                b = self.mapping.get(f"{robot_id}-{edge.target}")
                if not a or not b or a == b:
                    continue
                key = tuple(sorted((a, b)))
                current = self.edges.get(key)
                if current is None:
                    current = {"source": key[0], "target": key[1], "distance": edge.distance,
                               "traversal_count": edge.traversal_count, "observed_by": [robot_id]}
                    self.edges[key] = current
                else:
                    old = current["traversal_count"]
                    current["distance"] = (current["distance"] * old + edge.distance * edge.traversal_count) / (old + edge.traversal_count)
                    current["traversal_count"] = old + edge.traversal_count
                    if robot_id not in current["observed_by"]:
                        current["observed_by"].append(robot_id)
        public_nodes = [{
            "id": value["id"],
            "semantic_class": value["label"],
            "position": public_position(tuple(value["position"])),
            "navigation_position": public_position(
                tuple(value["navigation_position"])
                if value["navigation_position"] is not None else None
            ),
            "confidence": round(value["confidence"], 4),
            "observation_count": value["observation_count"],
        } for value in self.nodes.values()]
        public_edges = [{**edge, "distance": round(edge["distance"], 4),
                         "traversed": True}
                        for edge in self.edges.values()]
        return {"frame_id": "site_map", "nodes": public_nodes,
                "edges": public_edges, "source_mapping": dict(self.mapping)}
