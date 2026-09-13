import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("aaa_navigation")
    params = LaunchConfiguration("params_file")
    nodes = [
        ("mqtt_path_receiver", "mqtt_path_receiver"),
        ("scan_preprocessor", "scan_preprocessor"),
        ("vfh_controller", "vfh_controller"),
        ("safety_barrier", "safety_barrier"),
        ("cmd_vel_gateway", "aaa_cmd_gateway"),
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                package, "config", "jetson003_local_navigation.yaml"
            ),
        ),
        *[
            Node(
                package="aaa_navigation",
                executable=executable,
                name=name,
                output="screen",
                parameters=[params, {"use_sim_time": False}],
            )
            for executable, name in nodes
        ],
    ])
