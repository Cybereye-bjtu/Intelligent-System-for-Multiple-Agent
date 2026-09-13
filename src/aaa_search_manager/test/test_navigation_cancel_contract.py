from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_target_found_paths_cancel_navigation_before_target_publication():
    source = (PACKAGE_ROOT / 'aaa_search_manager' / 'mission_manager.py').read_text()
    legacy = source.index('self._cancel_navigation_tasks()', source.index('def _on_target_found'))
    legacy_publish = source.index('self.target_pose_pub.publish(msg)', legacy)
    global_handler = source.index('def _on_global_observation')
    global_cancel = source.index('self._cancel_navigation_tasks()', global_handler)
    global_publish = source.index('self.target_pose_pub.publish(pose)', global_cancel)
    assert legacy < legacy_publish
    assert global_cancel < global_publish
