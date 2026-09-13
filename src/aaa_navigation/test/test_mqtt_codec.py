import json
import math

import pytest

from aaa_navigation.mqtt_codec import (
    compact_vector,
    EnvelopeError,
    decode_envelope,
    decode_stamp,
    finite_ranges,
    fixed_length_floats,
    normalize_frame,
    normalize_epoch_seconds,
    resample_scan_ranges,
    scan_increment_from_endpoints,
    simple_twist_command,
    twist_publish_command,
)


def test_decode_real_edge_envelope():
    payload = json.dumps({
        "seq": 12,
        "ts": 123.5,
        "source": "/scan",
        "data": {"header": {"frame_id": "/laser"}},
    }).encode()
    sequence, timestamp, source, data = decode_envelope(payload)
    assert sequence == 12
    assert timestamp == 123.5
    assert source == "/scan"
    assert data["header"]["frame_id"] == "/laser"


def test_decodes_jetson003_array_envelope_and_millisecond_time():
    payload = json.dumps({
        "seq": 13,
        "ts": 1786109138426,
        "source": "jetson003",
        "data": [{"f": "odom", "c": "base_footprint"}],
    }).encode()
    sequence, timestamp, source, data = decode_envelope(payload)
    assert sequence == 13
    assert timestamp == pytest.approx(1786109138.426)
    assert source == "jetson003"
    assert data[0]["c"] == "base_footprint"


def test_normalizes_common_epoch_units():
    assert normalize_epoch_seconds(1786109138.426) == 1786109138.426
    assert normalize_epoch_seconds(1786109138426) == pytest.approx(
        1786109138.426
    )
    assert decode_stamp(1786109138426) == (1786109138, 426000118)


def test_twist_publish_command_matches_hyzx_edge_api():
    assert twist_publish_command(0.5, -0.25) == {
        "cmd": "publish",
        "topic": "/cmd_vel",
        "msg_type": "geometry_msgs/Twist",
        "data": {
            "linear": {"x": 0.5},
            "angular": {"z": -0.25},
        },
    }


def test_simple_twist_command_matches_jetson003_edge_api():
    assert simple_twist_command(0.1, -0.2) == {
        "linear_x": 0.1,
        "angular_z": -0.2,
    }


@pytest.mark.parametrize(
    ("stamp", "expected"),
    [
        ({"sec": 4, "nanosec": 5}, (4, 5)),
        ({"secs": 6, "nsecs": 7}, (6, 7)),
        ([8, 9], (8, 9)),
    ],
)
def test_decode_ros1_and_ros2_stamp_names(stamp, expected):
    assert decode_stamp(stamp) == expected


def test_rejects_invalid_envelope():
    with pytest.raises(EnvelopeError):
        decode_envelope(b'{"seq": 1, "ts": 2, "data": "bad"}')


def test_sanitizes_ranges_covariance_and_frames():
    ranges = finite_ranges([0.4, None, float("inf"), -1.0])
    assert ranges[0] == 0.4
    assert math.isinf(ranges[1])
    assert math.isinf(ranges[2])
    assert ranges[3] == -1.0
    assert fixed_length_floats([1, 2], 4) == [0.0, 0.0, 0.0, 0.0]
    assert normalize_frame("///base_footprint") == "base_footprint"


def test_decodes_hyzx_compact_scan_units_and_geometry():
    ranges = finite_ranges([581, 0, 2830], scale=0.001)
    assert ranges == [0.581, 0.0, 2.83]
    increment = scan_increment_from_endpoints(
        -2.268928, 2.268928, 1205
    )
    assert math.isclose(increment, 0.003769, abs_tol=1e-7)


def test_decodes_hyzx_compact_vectors():
    assert compact_vector([-0.0006, 0.0002, 7.2011], 3) == [
        -0.0006,
        0.0002,
        7.2011,
    ]
    assert compact_vector([0.0, 0.0], 3) == [0.0, 0.0, 0.0]


def test_normalizes_variable_lidar_beam_count():
    output = resample_scan_ranges([1.0, 2.0, 3.0], 5)
    assert output == [1.0, 1.0, 2.0, 3.0, 3.0]
