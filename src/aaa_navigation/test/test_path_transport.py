import math

import pytest

from aaa_navigation.path_transport import decode_path, encode_path, transform_xy


def test_path_round_trip():
    payload = encode_path([(1.0, 2.0), (3.0, 4.0)], "odom", 7, 10.0)
    sequence, timestamp, points = decode_path(payload, "odom")
    assert sequence == 7
    assert timestamp == 10.0
    assert points == [(1.0, 2.0), (3.0, 4.0)]


def test_path_rejects_wrong_frame_and_non_finite_values():
    payload = encode_path([(1.0, 2.0)], "odom", 1, 10.0)
    with pytest.raises(ValueError):
        decode_path(payload, "map")
    with pytest.raises(ValueError):
        encode_path([(math.nan, 0.0)], "odom", 1, 10.0)


def test_planar_transform():
    x, y = transform_xy(1.0, 0.0, 2.0, 3.0, math.pi / 2.0)
    assert x == pytest.approx(2.0)
    assert y == pytest.approx(4.0)
