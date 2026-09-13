from aaa_navigation.observation_confirmation import ConfirmationHistory


def observation(x):
    return {"position": [x, 0.0, 0.0]}


def test_confirmation_history_is_isolated_by_stream_and_prompt():
    history = ConfirmationHistory()

    topology = history.add(
        "topology", "school", observation(1.0),
        maximum_count=2, distance_limit=0.30,
    )
    mission = history.add(
        "mission", "school", observation(4.0),
        maximum_count=2, distance_limit=0.30,
    )
    topology = history.add(
        "topology", "school", observation(1.1),
        maximum_count=2, distance_limit=0.30,
    )
    mission = history.add(
        "mission", "school", observation(4.1),
        maximum_count=2, distance_limit=0.30,
    )

    assert len(topology) == 2
    assert len(mission) == 2
    assert history.count("topology", "school") == 2
    assert history.count("mission", "school") == 2


def test_position_jump_resets_only_the_matching_history():
    history = ConfirmationHistory()
    history.add(
        "topology", "school", observation(1.0),
        maximum_count=3, distance_limit=0.30,
    )
    history.add(
        "mission", "bottle", observation(2.0),
        maximum_count=3, distance_limit=0.30,
    )

    topology = history.add(
        "topology", "school", observation(3.0),
        maximum_count=3, distance_limit=0.30,
    )

    assert len(topology) == 1
    assert history.count("mission", "bottle") == 1
