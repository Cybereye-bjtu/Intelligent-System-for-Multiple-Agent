"""ROS-independent validation helpers for cloud velocity commands."""

from __future__ import annotations

from typing import Optional, Tuple


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_command_stamp(
    now_nanoseconds: int,
    stamp_nanoseconds: int,
    max_age_seconds: float,
    max_future_seconds: float,
    last_stamp_nanoseconds: Optional[int],
) -> Tuple[bool, str]:
    if stamp_nanoseconds <= 0:
        return False, "command has no source timestamp"
    age_seconds = (now_nanoseconds - stamp_nanoseconds) / 1e9
    if age_seconds > max_age_seconds:
        return False, "command source timestamp is stale"
    if age_seconds < -max_future_seconds:
        return False, "command source timestamp is in the future"
    if (
        last_stamp_nanoseconds is not None
        and stamp_nanoseconds <= last_stamp_nanoseconds
    ):
        return False, "command source timestamp is not increasing"
    return True, ""
