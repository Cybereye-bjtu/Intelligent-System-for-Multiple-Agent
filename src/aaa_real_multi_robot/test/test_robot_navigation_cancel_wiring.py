from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_each_namespaced_astar_receives_the_mission_cancel_topic():
    source = (PACKAGE_ROOT / 'launch' / 'robot_navigation.launch.py').read_text()
    assert '"cancel_topic": f"{prefix}/navigation/cancel"' in source
