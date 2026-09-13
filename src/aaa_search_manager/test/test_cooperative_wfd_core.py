import math

import numpy as np

from aaa_search_manager.cooperative_wfd_core import (
    Frontier,
    GoalProgressTracker,
    GridGeometry,
    align_grid_to_geometry,
    assign_frontiers,
    cell_to_world,
    detect_frontiers,
    path_costs,
    world_to_cell,
)


def explored_box(size=14):
    grid = np.full((size, size), -1, dtype=np.int16)
    grid[3:-3, 3:-3] = 0
    return grid


def test_wfd_detects_one_connected_boundary_from_reachable_free_space():
    grid = explored_box()
    frontiers, seeds = detect_frontiers(
        grid, [(6, 6)], resolution=0.05, min_frontier_length_m=0.20
    )
    assert seeds == ((6, 6),)
    assert len(frontiers) == 1
    assert frontiers[0].length_m >= 1.0
    assert all(grid[cell] == 0 for cell in frontiers[0].cells)


def test_small_frontier_noise_is_filtered_in_metric_units():
    grid = np.full((9, 9), 100, dtype=np.int16)
    grid[3:6, 3:6] = 0
    grid[2, 4] = -1
    small, _ = detect_frontiers(
        grid, [(4, 4)], resolution=0.05, min_frontier_length_m=0.20
    )
    kept, _ = detect_frontiers(
        grid, [(4, 4)], resolution=0.05, min_frontier_length_m=0.05
    )
    assert small == []
    assert len(kept) == 1


def test_multi_source_wfd_covers_two_disconnected_robot_regions():
    grid = np.full((20, 20), -1, dtype=np.int16)
    grid[2:7, 2:7] = 0
    grid[13:18, 13:18] = 0
    grid[7:13, :] = 100
    frontiers, seeds = detect_frontiers(
        grid, [(4, 4), (15, 15)], resolution=0.1, min_frontier_length_m=0.2
    )
    assert len(seeds) == 2
    assert len(frontiers) == 2


def test_path_costs_do_not_cross_unknown_or_lethal_cells():
    occupancy = np.zeros((7, 9), dtype=np.int16)
    occupancy[:, 4] = -1
    occupancy[3, 4] = 0
    esdf = np.zeros_like(occupancy)
    esdf[3, 4] = 100
    distances = path_costs(occupancy, esdf, (3, 1), resolution=0.1)
    assert math.isfinite(distances[3, 3])
    assert not math.isfinite(distances[3, 5])


def test_assignment_is_distinct_and_minimizes_total_path_cost():
    frontiers = [
        Frontier(cells=((0, 0),), length_m=1.0, centroid=(0.0, 0.0)),
        Frontier(cells=((0, 1),), length_m=1.0, centroid=(0.0, 1.0)),
        Frontier(cells=((0, 2),), length_m=1.0, centroid=(0.0, 2.0)),
    ]
    first = np.asarray([[1.0, 4.0, 8.0]])
    second = np.asarray([[2.0, 1.5, 9.0]])
    assigned = assign_frontiers({'hyzx001': first, 'jetson003': second}, frontiers)
    assert assigned['hyzx001'].frontier_index == 0
    assert assigned['jetson003'].frontier_index == 1


def test_one_reachable_frontier_assigns_only_the_lower_cost_robot():
    frontiers = [
        Frontier(cells=((0, 0),), length_m=1.0, centroid=(0.0, 0.0))
    ]
    assigned = assign_frontiers(
        {
            'hyzx001': np.asarray([[3.0]]),
            'jetson003': np.asarray([[1.0]]),
        },
        frontiers,
    )
    assert set(assigned) == {'jetson003'}


def test_assignment_skips_frontier_inside_navigation_tolerance():
    frontiers = [
        Frontier(cells=((0, 0),), length_m=1.0, centroid=(0.0, 0.0)),
        Frontier(cells=((0, 1),), length_m=1.0, centroid=(0.0, 1.0)),
    ]
    assigned = assign_frontiers(
        {'robot': np.asarray([[0.10, 0.60]])},
        frontiers,
        min_path_cost_m=0.25,
    )
    assert assigned['robot'].frontier_index == 1


def test_goal_progress_tracker_reports_arrival_and_stall():
    tracker = GoalProgressTracker(goal_tolerance_m=0.25, progress_timeout_s=10.0)
    assert tracker.set_goal('r1', (1.0, 0.0), (0.0, 0.0), now_s=0.0)
    assert not tracker.set_goal('r1', (1.1, 0.0), (0.0, 0.0), now_s=1.0)
    assert tracker.update({'r1': (0.4, 0.0)}, now_s=5.0) == {}
    assert tracker.update({'r1': (0.8, 0.0)}, now_s=6.0) == {'r1': 'reached'}

    assert tracker.set_goal('r2', (2.0, 0.0), (0.0, 0.0), now_s=0.0)
    assert tracker.update({'r2': (0.01, 0.0)}, now_s=10.0) == {'r2': 'stalled'}


def test_rotated_grid_world_cell_round_trip():
    geometry = GridGeometry(
        resolution=0.05, origin_x=-2.0, origin_y=1.0, origin_yaw=math.pi / 3.0
    )
    cell = (17, 23)
    assert world_to_cell(*cell_to_world(cell, geometry), geometry) == cell


def test_padded_esdf_is_aligned_back_to_shared_map_cells():
    source = np.full((8, 10), 100, dtype=np.int16)
    source[2:6, 3:7] = np.arange(16, dtype=np.int16).reshape(4, 4)
    source_geometry = GridGeometry(0.1, -0.3, -0.2)
    target_geometry = GridGeometry(0.1, 0.0, 0.0)
    aligned = align_grid_to_geometry(
        source, source_geometry, (4, 4), target_geometry
    )
    np.testing.assert_array_equal(aligned, source[2:6, 3:7])
