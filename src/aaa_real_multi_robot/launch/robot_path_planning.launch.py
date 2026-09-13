from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot_name").perform(context).strip("/")
    params_file = LaunchConfiguration("upstream_params").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context).lstrip("/")
    topology_file = LaunchConfiguration("topology_objects_file").perform(context)
    with open(params_file, encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    def params(name):
        return dict(document.get(name, {}).get("ros__parameters", {}))

    prefix = f"/{robot}"
    common = {"use_sim_time": False}
    return [
        Node(
            package="aaa_navigation", executable="esdf_mapper",
            namespace=robot, name="esdf_mapper", output="screen",
            parameters=[params("esdf_mapper"), common, {
                "map_topic": "/swarm/map", "output_topic": f"{prefix}/esdf_map",
                "map_subscription_transient_local": True,
            }],
        ),
        Node(
            package="aaa_navigation", executable="astar_planner",
            namespace=robot, name="astar_planner", output="screen",
            parameters=[params("astar_planner"), common, {
                "costmap_topic": f"{prefix}/esdf_map",
                "goal_topic": f"{prefix}/goal_pose",
                "path_topic": f"{prefix}/global_path",
                "cancel_topic": f"{prefix}/navigation/cancel",
                "map_frame": "site_map", "robot_frame": base_frame,
            }],
        ),
        Node(
            package="aaa_navigation", executable="kettle_navigator",
            namespace=robot, name="target_navigator", output="screen",
            parameters=[common, {
                "goal_topic": f"{prefix}/goal_pose",
                "target_pose_topic": f"{prefix}/target_pose",
                "status_topic": f"{prefix}/target_navigation/status",
                "visualization_topic": f"{prefix}/target_detection/image",
                "map_frame": "site_map", "robot_frame": base_frame,
                "capture_robot_frame": "base_footprint" if robot == "hyzx001" else "base_link",
                "worker_backend": "jetson_rpc",
                "black_root": "/data2/szhang/cybereye_nl_target_pose_black_vehicle",
                "mqtt_env_file": (
                    "/home/szhang/workspace/aaa_ros2_multirobot_find_target/"
                    + (".hyzx_mqtt.env" if robot == "hyzx001" else ".jetson003_mqtt.env")
                ),
                "device_id": robot,
                "prompt": "red water bottle", "standoff_distance": 0.30,
                "persistent_worker": True, "gpu_index": 0 if robot == "hyzx001" else 1,
                "continuous_detection": True, "semantic_mapping_mode": True,
                "topology_objects_file": topology_file,
                "semantic_targets_topic": "/semantic_topology/targets_config",
                "continuous_enabled_topic": "/search/exploration_enabled",
                "target_prompt_topic": "/search/target_prompt",
                "target_query_topic": "/search/target_query",
                "candidate_topic": f"/search/target_candidate/{robot}",
                "observation_topic": f"/search/target_observation/{robot}",
                "semantic_observation_topic": f"/semantic_topology/observation/{robot}",
                "detection_interval": 3.0, "auto_start": False,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name"),
        DeclareLaunchArgument("upstream_params"),
        DeclareLaunchArgument("base_frame"),
        DeclareLaunchArgument(
            "topology_objects_file",
            default_value=str(Path(get_package_share_directory("aaa_navigation")) / "config" / "semantic_topology_objects.json"),
        ),
        OpaqueFunction(function=launch_setup),
    ])


