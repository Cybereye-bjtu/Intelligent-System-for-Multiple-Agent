from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_approach_task_is_forwarded_to_existing_goal_pose_chain():
    source = (PACKAGE_ROOT / 'aaa_search_manager' / 'mission_manager.py').read_text()
    dispatch = source.index('def _dispatch_navigation')
    next_handler = source.index('def _on_navigation_status', dispatch)
    body = source[dispatch:next_handler]
    assert 'goal.pose = task.goal' in body
    assert 'self.goal_pubs[robot_id].publish(goal)' in body


def test_approach_arrival_monitor_advances_sequential_navigation():
    source = (PACKAGE_ROOT / 'aaa_search_manager' / 'mission_manager.py').read_text()
    monitor = source[source.index('def _monitor_navigation'):]
    assert 'lookup_transform(' in monitor
    assert 'distance > self.approach_goal_tolerance_m' in monitor
    assert 'NavigationStatus.SUCCEEDED' in monitor
    assert 'self._on_navigation_status(robot_id, message)' in monitor


def test_partial_approach_task_starts_then_waits_for_second_robot():
    source = (PACKAGE_ROOT / 'aaa_search_manager' / 'mission_manager.py').read_text()
    assert 'approach_task_collection_sec' in source
    assert 'self._start_approach_navigation()' in source
    assert 'self.second_task_wait_started = time.monotonic()' in source
    assert 'second_approach_wait_timeout_sec' in source
