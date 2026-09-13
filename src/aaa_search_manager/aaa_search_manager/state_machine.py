from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MissionState(str, Enum):
    IDLE = "IDLE"
    EXPLORING = "EXPLORING"
    TARGET_FOUND = "TARGET_FOUND"
    NAVIGATING_ROBOT_1 = "NAVIGATING_ROBOT_1"
    NAVIGATING_ROBOT_2 = "NAVIGATING_ROBOT_2"
    DONE = "DONE"
    FAILED = "FAILED"


@dataclass
class MissionSnapshot:
    state: MissionState
    query: str = ""


class MissionStateMachine:
    """Pure state machine; intentionally independent of ROS for easy testing."""

    def __init__(self) -> None:
        self._state = MissionState.IDLE
        self._query = ""

    @property
    def state(self) -> MissionState:
        return self._state

    @property
    def query(self) -> str:
        return self._query

    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot(state=self._state, query=self._query)

    def start(self, query: str) -> bool:
        query = query.strip()
        if not query:
            return False
        if self._state not in (MissionState.IDLE, MissionState.DONE):
            return False
        self._query = query
        self._state = MissionState.EXPLORING
        return True

    def target_found(self) -> bool:
        if self._state != MissionState.EXPLORING:
            return False
        self._state = MissionState.TARGET_FOUND
        return True

    def approach_ready(self) -> bool:
        if self._state != MissionState.TARGET_FOUND:
            return False
        self._state = MissionState.NAVIGATING_ROBOT_1
        return True

    def first_robot_succeeded(self) -> bool:
        if self._state != MissionState.NAVIGATING_ROBOT_1:
            return False
        self._state = MissionState.NAVIGATING_ROBOT_2
        return True

    def second_robot_succeeded(self) -> bool:
        if self._state != MissionState.NAVIGATING_ROBOT_2:
            return False
        self._state = MissionState.DONE
        return True

    def fail(self) -> bool:
        if self._state in (MissionState.IDLE, MissionState.DONE, MissionState.FAILED):
            return False
        self._state = MissionState.FAILED
        return True

    def reset(self) -> None:
        self._state = MissionState.IDLE
        self._query = ""
