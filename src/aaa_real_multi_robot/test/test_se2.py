import math

from aaa_real_multi_robot.se2 import apply, compose, distance, wrap


def test_compose_and_apply_agree():
    first = (1.0, 2.0, math.pi / 2.0)
    second = (2.0, 0.0, -math.pi / 2.0)
    result = compose(first, second)
    assert all(math.isclose(actual, expected, abs_tol=1e-12) for actual, expected in zip(result, (1.0, 4.0, 0.0)))
    assert all(math.isclose(actual, expected, abs_tol=1e-12) for actual, expected in zip(apply(first, 2.0, 0.0), (1.0, 4.0)))


def test_distance_wraps_heading():
    translation, rotation = distance((0.0, 0.0, math.pi - 0.1), (0.0, 0.0, -math.pi + 0.1))
    assert translation == 0.0
    assert math.isclose(rotation, 0.2)
    assert math.isclose(wrap(3.0 * math.pi), math.pi)
