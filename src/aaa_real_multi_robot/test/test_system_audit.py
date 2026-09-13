import pytest

from sensor_msgs.msg import LaserScan

from aaa_real_multi_robot.system_audit import SystemAudit, diagnostic_level_value


def test_diagnostic_level_accepts_integer_and_single_byte():
    assert diagnostic_level_value(0) == 0
    assert diagnostic_level_value(2) == 2
    assert diagnostic_level_value(b"\x00") == 0
    assert diagnostic_level_value(b"\x02") == 2


def test_diagnostic_level_rejects_invalid_byte_length():
    with pytest.raises(ValueError):
        diagnostic_level_value(b"")
    with pytest.raises(ValueError):
        diagnostic_level_value(b"\x00\x01")


def test_scan_health_rejects_all_zero_scan():
    node = SystemAudit.__new__(SystemAudit)
    node.received = {}
    node.scan_health = {}
    message = LaserScan()
    message.range_min = 0.05
    message.range_max = 20.0
    message.ranges = [0.0] * 100
    node._scan_callback("/robot/scan", message)
    assert node.scan_health["/robot/scan"] == (0, 100)


def test_scan_health_counts_only_finite_in_range_values():
    node = SystemAudit.__new__(SystemAudit)
    node.received = {}
    node.scan_health = {}
    message = LaserScan()
    message.range_min = 0.05
    message.range_max = 10.0
    message.ranges = [0.0, float("nan"), float("inf"), 0.5, 10.0, 11.0]
    node._scan_callback("/robot/scan", message)
    assert node.scan_health["/robot/scan"] == (2, 6)
