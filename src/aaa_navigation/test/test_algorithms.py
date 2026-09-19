import math

import numpy as np

from aaa_navigation.algorithms import (
    astar_grid,
    choose_vfh_heading,
    choose_vfh_recovery_heading,
    corner_aware_target,
    euclidean_distance_transform,
    fuzzy_velocity,
    regulated_angular_velocity,
    resample_polyline,
    simplify_path,
    transform_polar_points,
    update_alignment_state,
)


def test_distance_transform_single_obstacle():
    obstacles = np.zeros((5, 5), dtype=bool)
    obstacles[2, 2] = True
    distance = euclidean_distance_transform(obstacles)
    assert distance[2, 2] == 0.0
    assert distance[2, 4] == 2.0
    assert np.isclose(distance[0, 0], math.sqrt(8.0))


def test_astar_avoids_wall_and_simplifies():
    costs = np.zeros((10, 10), dtype=np.int16)
    costs[1:9, 5] = 100
    costs[5, 5] = 0
    path = astar_grid(costs, (1, 5), (8, 5), lethal_cost=96)
    assert path
    assert all(costs[y, x] < 96 for x, y in path)
    simpler = simplify_path(path, costs, 96)
    assert simpler[0] == (1, 5)
    assert simpler[-1] == (8, 5)


def test_astar_projects_inflated_start():
    costs = np.zeros((20, 20), dtype=np.int16)
    costs[8:13, 8:13] = 100
    path = astar_grid(costs, (10, 10), (18, 10), lethal_cost=96)
    assert path
    assert costs[path[0][1], path[0][0]] < 96
    assert path[-1] == (18, 10)


def test_corner_target_is_finite():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]
    target = corner_aware_target(
        path,
        (0.0, 0.0),
        lookahead=0.8,
        corner_search=1.5,
        corner_threshold=math.radians(20.0),
        shift_gain=0.2,
        shift_max=0.25,
    )
    assert np.all(np.isfinite(target))
    assert math.hypot(*target) > 0.5


def test_resample_polyline_preserves_corner_and_spacing():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    sampled = resample_polyline(points, 0.25)
    assert points[1] in sampled
    assert sampled[0] == points[0]
    assert sampled[-1] == points[-1]
    assert max(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(sampled, sampled[1:])
    ) <= 0.25 + 1e-9


def test_vfh_selects_observed_open_valley():
    histogram = np.ones(120)
    observed = np.zeros(120, dtype=bool)
    histogram[55:66] = 0.0
    observed[55:66] = True
    heading = choose_vfh_heading(
        histogram,
        observed,
        target_angle=0.0,
        previous_angle=0.0,
        obstacle_threshold=0.5,
        sector_width=2.0 * math.pi / 120,
        minimum_valley_width=5,
        margin_sectors=1,
        heading_weight=5.0,
        previous_weight=2.0,
        clearance_weight=1.0,
    )
    assert heading is not None
    assert abs(heading) < 0.2


def test_vfh_recovery_never_selects_unobserved_blind_sector():
    histogram = np.ones(120)
    observed = np.zeros(120, dtype=bool)
    observed[45:75] = True
    histogram[69] = 0.2
    heading = choose_vfh_recovery_heading(
        histogram,
        observed,
        target_angle=math.pi,
        previous_angle=0.0,
        sector_width=2.0 * math.pi / 120,
    )
    assert heading is not None
    index = int((heading + math.pi) / (2.0 * math.pi / 120))
    assert observed[index]


def test_fuzzy_velocity_slows_for_turns_and_obstacles():
    fast, _ = fuzzy_velocity(4.0, 0.0, 0.5, 1.0)
    near, _ = fuzzy_velocity(0.3, 0.0, 0.5, 1.0)
    turn, angular = fuzzy_velocity(4.0, math.pi / 2.0, 0.5, 1.0)
    assert fast > near
    assert turn <= near
    assert angular > 0.0


def test_transform_polar_points_applies_laser_offset():
    distances, angles = transform_polar_points(
        np.array([1.0, 1.0]),
        np.array([0.0, math.pi / 2.0]),
        translation_x=0.05,
        translation_y=0.0,
        yaw=0.0,
    )
    assert np.isclose(distances[0], 1.05)
    assert np.isclose(angles[0], 0.0)
    assert np.isclose(distances[1], math.hypot(0.05, 1.0))
    assert angles[1] < math.pi / 2.0


def test_alignment_state_uses_hysteresis():
    assert update_alignment_state(
        False, math.radians(30.0), math.radians(25.0), math.radians(8.0)
    )
    assert update_alignment_state(
        True, math.radians(10.0), math.radians(25.0), math.radians(8.0)
    )
    assert not update_alignment_state(
        True, math.radians(5.0), math.radians(25.0), math.radians(8.0)
    )
    # Alignment follows the path-heading error, not a temporarily small VFH
    # valley heading from a partial-field-of-view lidar.
    assert update_alignment_state(
        False, math.radians(80.0), math.radians(25.0), math.radians(8.0)
    )


def test_regulated_angular_velocity_brakes_and_has_deadband():
    assert regulated_angular_velocity(
        math.radians(2.0), 0.2, 0.8, math.radians(3.0), 0.8
    ) == 0.0
    far = regulated_angular_velocity(
        math.radians(45.0), 0.2, 0.8, math.radians(3.0), 0.8
    )
    near = regulated_angular_velocity(
        math.radians(5.0), 0.2, 0.8, math.radians(3.0), 0.8
    )
    reverse = regulated_angular_velocity(
        math.radians(-5.0), 0.2, 0.8, math.radians(3.0), 0.8
    )
    assert 0.0 < near < far <= 0.2
    assert np.isclose(reverse, -near)
