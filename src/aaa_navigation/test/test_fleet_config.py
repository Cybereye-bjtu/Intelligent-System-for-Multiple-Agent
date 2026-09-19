from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config"


def load_parameters(filename, node):
    document = yaml.safe_load((CONFIG / filename).read_text())
    return document[node]["ros__parameters"]


def test_cloud_configs_select_each_real_vehicle_protocol():
    hyzx = load_parameters("hyzx_mqtt_cloud.yaml", "mqtt_cloud_bridge")
    jetson = load_parameters(
        "jetson003_mqtt_cloud.yaml", "mqtt_cloud_bridge"
    )
    assert hyzx["hardware_id"] == "hyzx001"
    assert hyzx["command_payload_format"] == "ros_publish"
    assert hyzx["base_frame"] == "base_footprint"
    assert hyzx["frame_aliases"] == [""]
    assert jetson["hardware_id"] == "jetson003"
    assert jetson["command_payload_format"] == "simple_twist"
    assert jetson["base_frame"] == "base_link"
    assert set(jetson["frame_aliases"]) == {
        "base_footprint:=base_link",
        "lidar_frame:=laser_link",
    }


def test_navigation_and_edge_frames_match_per_robot():
    cases = (
        ("hyzx_mqtt_cloud.yaml", "hyzx_edge.yaml", "base_footprint"),
        ("jetson003_mqtt_cloud.yaml", "jetson003_edge.yaml", "base_link"),
    )
    for cloud_file, edge_file, base_frame in cases:
        cloud = yaml.safe_load((CONFIG / cloud_file).read_text())
        edge = yaml.safe_load((CONFIG / edge_file).read_text())
        for node in ("astar_planner", "vfh_controller", "safety_barrier"):
            assert cloud[node]["ros__parameters"]["robot_frame"] == base_frame
        assert (
            edge["safety_barrier"]["ros__parameters"]["robot_frame"]
            == base_frame
        )


def test_all_motion_outputs_remain_locked_at_start():
    for filename in ("hyzx_edge.yaml", "jetson003_edge.yaml"):
        document = yaml.safe_load((CONFIG / filename).read_text())
        assert not document["edge_command_receiver"]["ros__parameters"][
            "armed_on_start"
        ]
        assert not document["aaa_cmd_gateway"]["ros__parameters"][
            "enabled_on_start"
        ]
    jetson_mqtt = yaml.safe_load(
        (CONFIG / "jetson003_mqtt_edge.yaml").read_text()
    )
    assert not jetson_mqtt["aaa_cmd_gateway"]["ros__parameters"][
        "enabled_on_start"
    ]


def test_target_navigator_uses_local_dual_path_models():
    source = (
        CONFIG.parent / "aaa_navigation" / "kettle_navigator.py"
    ).read_text()
    root = "/home/szhang/workspace/cybereye_system/cybereye_nl_target_pose_white_vehicle"
    assert f'"cybereye_root": "{root}"' in source
    assert f'"sam3_runner": "{root}/run_sam3_env.sh"' in source
    assert '"sam3_checkpoint": "/data2/szhang/model/sam3/sam3.pt"' in source
    assert '"ocr_detection_model": "/data2/szhang/model/PP-OCRv6_medium_det"' in source
    assert '"ocr_recognition_model": "/data2/szhang/model/PP-OCRv6_medium_rec"' in source
    assert '"textregion_checkpoint": "/data2/szhang/model/ViT-L-16-SigLIP2-256/open_clip_pytorch_model.bin"' in source
