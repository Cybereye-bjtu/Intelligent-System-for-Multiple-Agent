from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import yaml


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot_name").perform(context).strip("/")
    upstream_params = LaunchConfiguration("upstream_params").perform(context)
    gate_params = LaunchConfiguration("gate_params").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context).lstrip("/")
    topology_objects_file = LaunchConfiguration("topology_objects_file").perform(context)
    prefix = f"/{robot}"
    common = {"use_sim_time": False}
    with open(upstream_params, encoding="utf-8") as stream:
        upstream_document = yaml.safe_load(stream) or {}

    def node_params(name):
        return dict(upstream_document.get(name, {}).get("ros__parameters", {}))
    return [
        Node(
            package="aaa_navigation",
            executable="scan_preprocessor",
            namespace=robot,
            name="scan_preprocessor",
            output="screen",
            parameters=[
                node_params("scan_preprocessor"),
                common,
                {"input_topic": f"{prefix}/scan", "output_topic": f"{prefix}/scan_filtered"},
            ],
        ),
        Node(
            package="aaa_navigation",
            executable="esdf_mapper",
            namespace=robot,
            name="esdf_mapper",
            output="screen",
            parameters=[
                node_params("esdf_mapper"),
                common,
                {
                    "map_topic": "/swarm/map",
                    "output_topic": f"{prefix}/esdf_map",
                    "map_subscription_transient_local": True,
                },
            ],
        ),
        Node(
            package="aaa_navigation",
            executable="astar_planner",
            namespace=robot,
            name="astar_planner",
            output="screen",
            parameters=[
                node_params("astar_planner"),
                common,
                {
                    "costmap_topic": f"{prefix}/esdf_map",
                    "goal_topic": f"{prefix}/goal_pose",
                    "path_topic": f"{prefix}/global_path",
                    "cancel_topic": f"{prefix}/navigation/cancel",
                    "map_frame": "site_map",
                    "robot_frame": base_frame,
                },
            ],
        ),
        Node(
            package="aaa_navigation",
            executable="vfh_controller",
            namespace=robot,
            name="vfh_controller",
            output="screen",
            parameters=[
                node_params("vfh_controller"),
                common,
                {
                    "scan_topic": f"{prefix}/scan_filtered",
                    "path_topic": f"{prefix}/global_path",
                    "nominal_cmd_topic": f"{prefix}/cmd_vel_nominal",
                    "map_frame": "site_map",
                    "robot_frame": base_frame,
                    "max_linear_velocity": 0.10,
                    # Both deployed chassis need 0.10 m/s to overcome their
                    # measured motor deadband.
                    "min_effective_linear_velocity": 0.10,
                    "max_angular_velocity": 0.20,
                },
            ],
            remappings=[("/vfh/debug/markers", f"{prefix}/vfh/debug/markers")],
        ),
        Node(
            package="aaa_navigation",
            executable="safety_barrier",
            namespace=robot,
            name="safety_barrier",
            output="screen",
            parameters=[
                node_params("safety_barrier"),
                common,
                {
                    "scan_topic": f"{prefix}/scan_filtered",
                    "odom_topic": f"{prefix}/odom",
                    "nominal_cmd_topic": f"{prefix}/cmd_vel_nominal",
                    "output_cmd_topic": f"{prefix}/cmd_vel_safe",
                    "robot_frame": base_frame,
                    "max_linear_velocity": 0.10,
                    "max_angular_velocity": 0.20,
                },
            ],
            remappings=[("/safety/debug/markers", f"{prefix}/safety/debug/markers")],
        ),
        Node(
            package="aaa_real_multi_robot",
            executable="lease_gate",
            namespace=robot,
            name="lease_gate",
            output="screen",
            parameters=[
                gate_params,
                common,
                {
                    "input_topic": f"{prefix}/cmd_vel_safe",
                    "output_topic": f"{prefix}/cmd_vel_leased",
                    "lease_topic": f"{prefix}/fleet/lease",
                },
            ],
        ),
        Node(
            package="aaa_navigation",
            executable="cloud_command_sender",
            namespace=robot,
            name="cloud_command_sender",
            output="screen",
            parameters=[
                node_params("cloud_command_sender"),
                common,
                {
                    "input_topic": f"{prefix}/cmd_vel_leased",
                    "output_topic": f"{prefix}/cmd_vel_cloud",
                    "heartbeat_topic": f"{prefix}/cloud_heartbeat",
                    "max_linear_velocity": 0.10,
                    "max_angular_velocity": 0.20,
                    "base_frame": base_frame,
                },
            ],
        ),
        Node(
                package="aaa_navigation",
                executable="kettle_navigator",
                namespace=robot,
                name="target_navigator",
                output="screen",
                parameters=[
                    common,
                    {
                        "goal_topic": f"{prefix}/goal_pose",
                        "target_pose_topic": f"{prefix}/target_pose",
                        "status_topic": f"{prefix}/target_navigation/status",
                        "visualization_topic": f"{prefix}/target_detection/image",
                        "map_frame": "site_map",
                        "robot_frame": base_frame,
                        # Detection runs on the central host, but the Jetson RGB-D
                        # snapshot probe resolves TF in the vehicle-local ROS graph.
                        # Keep robot_frame prefixed for central navigation while using
                        # the vehicle's unprefixed base frame for remote capture.
                        "capture_robot_frame": (
                            "base_footprint" if robot == "hyzx001" else "base_link"
                        ),
                        "worker_backend": (
                            "hyzx_snapshot" if robot == "hyzx001" else "jetson_rpc"
                        ),
                        "prompt": "red water bottle",
                        "standoff_distance": 0.30,
                        "persistent_worker": True,
                        "gpu_index": 0 if robot == "hyzx001" else 1,
                        "continuous_detection": True,
                        "semantic_mapping_mode": True,
                        "topology_objects_file": topology_objects_file,
                        "semantic_targets_topic": "/semantic_topology/targets_config",
                        "continuous_enabled_topic": "/search/exploration_enabled",
                        "target_prompt_topic": "/search/target_prompt",
                        "target_query_topic": "/search/target_query",
                        "candidate_topic": f"/search/target_candidate/{robot}",
                        "observation_topic": f"/search/target_observation/{robot}",
                        "semantic_observation_topic": f"/semantic_topology/observation/{robot}",
                        "detection_interval": 3.0,
                        "auto_start": False,
                    },
                ],
            ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name"),
            DeclareLaunchArgument("upstream_params"),
            DeclareLaunchArgument("gate_params"),
            DeclareLaunchArgument("base_frame"),
            DeclareLaunchArgument(
                "topology_objects_file",
                default_value=str(
                    Path(get_package_share_directory("aaa_navigation"))
                    / "config"
                    / "semantic_topology_objects.json"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
