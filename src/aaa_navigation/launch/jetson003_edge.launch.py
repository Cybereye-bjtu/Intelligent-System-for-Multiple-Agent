import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("aaa_navigation")
    params = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                package, "config", "jetson003_edge.yaml"
            ),
        ),
        Node(
            package="aaa_navigation",
            executable="scan_preprocessor",
            name="scan_preprocessor",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
        ),
        Node(
            package="aaa_navigation",
            executable="edge_command_receiver",
            name="edge_command_receiver",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
        ),
        Node(
            package="aaa_navigation",
            executable="safety_barrier",
            name="safety_barrier",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
        ),
        Node(
            package="aaa_navigation",
            executable="cmd_vel_gateway",
            name="aaa_cmd_gateway",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
        ),
    ])
