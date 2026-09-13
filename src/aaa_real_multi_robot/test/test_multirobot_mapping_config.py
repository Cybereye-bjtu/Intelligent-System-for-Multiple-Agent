from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parameters(filename):
    content = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    return content["/**/slam_toolbox"]["ros__parameters"]


def test_both_slam_nodes_exchange_localized_scans_without_peer_tf():
    hyzx = _parameters("hyzx001_slam.yaml")
    jetson = _parameters("jetson003_slam.yaml")
    assert hyzx["scan_share_topic"] == "/localized_scan"
    assert jetson["scan_share_topic"] == "/localized_scan"
    assert hyzx["publish_peer_transforms"] is False
    assert jetson["publish_peer_transforms"] is False
    assert hyzx["use_known_initial_poses"] is True
    assert jetson["use_known_initial_poses"] is True
    assert hyzx["initial_poses_xy_yaw"] == [0.0, 0.30, 0.0, 0.0, 0.0, 0.0]
    assert jetson["initial_poses_xy_yaw"] == hyzx["initial_poses_xy_yaw"]
    assert hyzx["scan_topic"] == "scan_mapping_filtered"
    assert jetson["scan_topic"] == "scan_mapping_filtered"
    assert hyzx["map_frame"] == "hyzx001/map"
    assert jetson["map_frame"] == "jetson003/map"


def test_map_fuser_owns_navigation_map_and_map_merge_is_reference_only():
    merge = yaml.safe_load((PACKAGE_ROOT / "config" / "map_merge.yaml").read_text())
    merge_params = merge["/**/map_merge"]["ros__parameters"]
    assert merge_params["known_init_poses"] is True
    assert merge_params["merged_map_topic"] == "/swarm/map_mexplore"
    assert merge_params["world_frame"] == "site_map"
    assert merge_params["/hyzx001/map_merge/init_pose_yaw"] == 0.0
    assert merge_params["/hyzx001/map_merge/init_pose_y"] == 0.30
    assert merge_params["/jetson003/map_merge/init_pose_y"] == 0.0

    fleet = yaml.safe_load((PACKAGE_ROOT / "config" / "fleet.yaml").read_text())
    fuser_params = fleet["/**/map_fuser"]["ros__parameters"]
    assert fuser_params["output_topic"] == "/swarm/map"
    alignment = fleet["/**/alignment_manager"]["ros__parameters"]
    assert alignment["initial_poses_xy_yaw_deg"] == [
        0.0, 0.30, 0.0, 0.0, 0.0, 0.0
    ]


def test_old_cslam_runtime_files_are_absent():
    assert not (PACKAGE_ROOT / "launch" / "swarm_slam.launch.py").exists()
    assert not (PACKAGE_ROOT / "config" / "swarm_slam_lidar.yaml").exists()
    assert not (PACKAGE_ROOT / "aaa_real_multi_robot" / "cslam_reference_adapter.py").exists()


def test_vehicle_specific_scan_timestamp_policies():
    expected = {"hyzx001": True, "jetson003": True}
    for robot, synchronize in expected.items():
        document = yaml.safe_load(
            (PACKAGE_ROOT / "config" / f"{robot}_adapter.yaml").read_text()
        )
        parameters = document["/**"]["ros__parameters"]
        assert parameters["zero_initial_odom"] is True
        assert parameters["synchronize_scan_to_odom"] is synchronize
        assert parameters["max_scan_odom_sync_age"] == 0.50


def test_hyzx_uses_legacy_receipt_time_and_mapping_range():
    cloud = yaml.safe_load(
        (
            PACKAGE_ROOT.parent
            / "aaa_navigation"
            / "config"
            / "hyzx_mqtt_cloud.yaml"
        ).read_text()
    )
    bridge = cloud["mqtt_cloud_bridge"]["ros__parameters"]
    assert bridge["use_envelope_timestamp"] is True
    assert bridge["max_envelope_age"] == 1.50
    launch_source = (
        PACKAGE_ROOT / "launch" / "cloud_ingress.launch.py"
    ).read_text()
    assert '"use_envelope_timestamp": False' in launch_source
    mapping = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hyzx001_mapping_scan.yaml").read_text()
    )
    assert mapping["/**/mapping_scan_preprocessor"]["ros__parameters"][
        "range_max"
    ] == 20.0


def test_rviz_defaults_to_clean_navigation_layers():
    document = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "aaa_real_multi_robot.rviz").read_text()
    )
    displays = document["Visualization Manager"]["Displays"]
    by_name = {display["Name"]: display for display in displays}
    assert document["Visualization Manager"]["Global Options"][
        "Fixed Frame"
    ] == "site_map"
    assert by_name["1 Unified Navigation Map"]["Enabled"] is True
    assert by_name["2 hyzx001 Mapping Scan"]["Enabled"] is True
    assert by_name["3 jetson003 Mapping Scan"]["Enabled"] is True
    for name, display in by_name.items():
        if name.startswith("Debug -"):
            assert display["Enabled"] is False
