from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_astar_cancel_clears_goal_and_publishes_empty_path():
    source = (PACKAGE_ROOT / 'aaa_navigation' / 'astar_planner.py').read_text()
    assert '"cancel_topic": "/navigation/cancel"' in source
    callback = source[source.index('def cancel_callback'):source.index('def _robot_position')]
    assert 'self.goal = None' in callback
    assert 'self._publish_empty()' in callback
