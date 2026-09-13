from aaa_search_manager.state_machine import MissionState, MissionStateMachine


def test_happy_path():
    sm = MissionStateMachine()
    assert sm.state == MissionState.IDLE
    assert sm.start('find the black vehicle')
    assert sm.state == MissionState.EXPLORING
    assert sm.query == 'find the black vehicle'
    assert sm.target_found()
    assert sm.state == MissionState.TARGET_FOUND
    assert sm.approach_ready()
    assert sm.state == MissionState.NAVIGATING_ROBOT_1
    assert sm.first_robot_succeeded()
    assert sm.state == MissionState.NAVIGATING_ROBOT_2
    assert sm.second_robot_succeeded()
    assert sm.state == MissionState.DONE


def test_empty_query_is_rejected():
    sm = MissionStateMachine()
    assert not sm.start('   ')
    assert sm.state == MissionState.IDLE


def test_second_query_is_rejected_while_active():
    sm = MissionStateMachine()
    assert sm.start('target a')
    assert not sm.start('target b')
    assert sm.query == 'target a'
    assert sm.state == MissionState.EXPLORING


def test_target_found_only_from_exploring():
    sm = MissionStateMachine()
    assert not sm.target_found()
    assert sm.state == MissionState.IDLE


def test_reset_returns_to_idle():
    sm = MissionStateMachine()
    sm.start('target')
    sm.target_found()
    sm.reset()
    assert sm.state == MissionState.IDLE
    assert sm.query == ''


def test_navigation_failure_is_terminal_until_reset():
    sm = MissionStateMachine()
    sm.start('target')
    sm.target_found()
    sm.approach_ready()
    assert sm.fail()
    assert sm.state == MissionState.FAILED
    assert not sm.first_robot_succeeded()
    assert not sm.start('another target')
