from aaa_navigation.cloud_protocol import validate_command_stamp


def test_accepts_fresh_increasing_timestamp():
    valid, reason = validate_command_stamp(
        now_nanoseconds=10_000_000_000,
        stamp_nanoseconds=9_900_000_000,
        max_age_seconds=0.30,
        max_future_seconds=0.10,
        last_stamp_nanoseconds=9_800_000_000,
    )
    assert valid
    assert reason == ""


def test_rejects_stale_timestamp():
    valid, reason = validate_command_stamp(
        now_nanoseconds=10_000_000_000,
        stamp_nanoseconds=9_600_000_000,
        max_age_seconds=0.30,
        max_future_seconds=0.10,
        last_stamp_nanoseconds=None,
    )
    assert not valid
    assert "stale" in reason


def test_rejects_future_timestamp():
    valid, reason = validate_command_stamp(
        now_nanoseconds=10_000_000_000,
        stamp_nanoseconds=10_200_000_000,
        max_age_seconds=0.30,
        max_future_seconds=0.10,
        last_stamp_nanoseconds=None,
    )
    assert not valid
    assert "future" in reason


def test_rejects_replayed_timestamp():
    valid, reason = validate_command_stamp(
        now_nanoseconds=10_000_000_000,
        stamp_nanoseconds=9_950_000_000,
        max_age_seconds=0.30,
        max_future_seconds=0.10,
        last_stamp_nanoseconds=9_950_000_000,
    )
    assert not valid
    assert "not increasing" in reason
