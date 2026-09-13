import math

from aaa_search_manager.semantic_topology_core import GraphFusion, RobotGraph


def graph(robot='r1', **kwargs):
    kwargs.setdefault('min_observations', 3)
    return RobotGraph(robot, observation_interval=0.5,
                      min_observation_baseline=0.15, **kwargs)


def confirm(g, label, position, robot_positions):
    node = None
    for stamp, robot_position in enumerate(robot_positions):
        node = g.observe(label, position, 0.8, robot_position,
                         float(stamp), lambda _: True)
    assert node.confirmed
    return node


def test_merge_confirmation_baseline_and_navigation_position():
    g = graph()
    assert g.observe('chair', (1.0, 1.0), 0.49, (0.0, 1.0), 0.0, lambda _: True) is None
    node = g.observe('chair', (1.0, 1.0), 0.8, (0.0, 1.0), 0.0, lambda _: True)
    g.observe('chair', (1.05, 1.0), 0.9, (0.05, 1.0), 0.2, lambda _: True)
    assert node.observation_count == 1
    g.observe('chair', (1.02, 1.0), 0.9, (0.1, 1.0), 1.0, lambda _: True)
    g.observe('chair', (1.01, 1.0), 0.9, (0.3, 1.0), 2.0, lambda _: True)
    assert node.confirmed and node.observation_count == 3
    assert math.isclose(node.navigation_position[1], 1.0, abs_tol=1e-6)
    assert node.navigation_position[0] < node.position[0]
    public = g.public(confirmed_only=True)['nodes'][0]
    assert public['id'] == 'N1'
    assert public['semantic_class'] == 'chair'
    assert public['position'] == {'x': public['position']['x'], 'y': 1.0, 'z': 0.0}
    assert set(public) == {
        'id', 'semantic_class', 'position', 'navigation_position',
        'confidence', 'observation_count',
    }


def test_single_observation_mode_confirms_without_baseline():
    g = graph(min_observations=1)
    node = g.observe('school', (1.0, 1.0), 0.8, (0.5, 1.0),
                     0.0, lambda _: True)
    assert node.confirmed
    assert node.observation_count == 1
    assert node.navigation_position is not None


def test_default_confidence_threshold_accepts_point_five_only():
    g = RobotGraph('r1', min_observations=1)
    assert g.observe('bank', (1.0, 1.0), 0.499, (0.5, 1.0),
                     0.0, lambda _: True) is None
    node = g.observe('bank', (1.0, 1.0), 0.50, (0.5, 1.0),
                     1.0, lambda _: True)
    assert node is not None and node.confirmed


def test_occupied_navigation_position_prevents_visit():
    g = graph()
    node = None
    for stamp, robot in enumerate(((0.0, 0.0), (0.2, 0.0), (0.4, 0.0))):
        node = g.observe('door', (1.0, 0.0), 0.9, robot, stamp, lambda _: False)
    assert node.confirmed and node.navigation_position is None
    g.update_pose(1.0, 0.0, 0.0)
    assert g.last_visited_node is None


def test_edge_requires_real_valid_trajectory_and_is_undirected():
    g = graph(localization_jump_limit=0.25, sample_distance=0.04)
    a = confirm(g, 'chair', (0.0, 0.0), ((0.3, 0.0), (0.5, 0.0), (0.7, 0.0)))
    b = confirm(g, 'door', (1.5, 0.0), ((0.7, 0.0), (0.5, 0.0), (0.3, 0.0)))
    # Use the actual computed navigation positions.
    g.update_pose(*a.navigation_position, 0.0)
    g.update_pose(a.navigation_position[0] + 0.5, 0.0, 0.0)  # jump invalidates
    g.update_pose(*b.navigation_position, 0.0)
    assert not g.edges
    g.update_pose(*a.navigation_position, 0.0)  # leave B after the forced jump
    g.update_pose(*a.navigation_position, 0.0)  # enter A and reset
    for x in (0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00, 1.10):
        g.update_pose(x, 0.0, 0.0)
    g.update_pose(*b.navigation_position, 0.0)
    assert len(g.edges) == 1
    edge = next(iter(g.edges.values()))
    assert {edge.source, edge.target} == {a.id, b.id}
    assert edge.distance >= 0.30
    public_edge = g.public(confirmed_only=True)['edges'][0]
    assert {public_edge['source'], public_edge['target']} == {'N1', 'N2'}


def test_fusion_keeps_mapping_and_only_fuses_traversed_edges():
    first, second = graph('hyzx001'), graph('jetson003')
    a = confirm(first, 'chair', (1.0, 1.0), ((0, 1), (.2, 1), (.4, 1)))
    b = confirm(second, 'chair', (1.1, 1.0), ((0, 1), (.2, 1), (.4, 1)))
    fusion = GraphFusion(0.20)
    result = fusion.update({'hyzx001': first, 'jetson003': second})
    assert len(result['nodes']) == 1
    assert result['nodes'][0]['id'] == 'N1'
    assert result['nodes'][0]['semantic_class'] == 'chair'
    assert result['nodes'][0]['position']['z'] == 0.0
    assert result['source_mapping'][f'hyzx001-{a.id}'] == result['source_mapping'][f'jetson003-{b.id}']
    assert result['edges'] == []
    again = fusion.update({'hyzx001': first, 'jetson003': second})
    assert again['nodes'][0]['observation_count'] == 6
