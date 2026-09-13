import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package = get_package_share_directory("aaa_real_multi_robot")
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        raise RuntimeError("central_localization.launch.py requires map:=/absolute/path/map.yaml")
    amcl_config = os.path.join(package, "config", "amcl.yaml")
    fleet_config = os.path.join(package, "config", "fleet.yaml")
    nodes = [
        Node(
            package="aaa_real_multi_robot",
            executable="fleet_safety_manager",
            name="fleet_safety_manager",
            output="screen",
            parameters=[{"default_emergency_stop": True}],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="swarm_map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml, "frame_id": "site_map", "topic_name": "map", "use_sim_time": False}],
            remappings=[("map", "/swarm/map")],
        )
    ]
    robot_frames = {
        "hyzx001": ("hyzx001/odom", "hyzx001/base_footprint"),
        "jetson003": ("jetson003/odom", "jetson003/base_link"),
    }
    for robot, (odom_frame, base_frame) in robot_frames.items():
        nodes.append(
            Node(
                package="nav2_amcl",
                executable="amcl",
                namespace=robot,
                name="amcl",
                output="screen",
                parameters=[amcl_config, {"odom_frame_id": odom_frame, "base_frame_id": base_frame}],
                remappings=[("map", "/swarm/map"), ("scan", f"/{robot}/scan")],
            )
        )
        nodes.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=robot,
                name="amcl_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["amcl"],
                        "use_sim_time": False,
                    }
                ],
            )
        )
    nodes.extend(
        [
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="map_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["swarm_map_server"],
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="aaa_real_multi_robot",
                executable="fleet_coordinator",
                name="fleet_coordinator",
                output="screen",
                parameters=[fleet_config],
            ),
        ]
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
