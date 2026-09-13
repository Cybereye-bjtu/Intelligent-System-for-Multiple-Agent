import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory("aaa_real_multi_robot")
    return LaunchDescription(
        [
            Node(
                package="aaa_real_multi_robot",
                executable="alignment_manager",
                name="alignment_manager",
                output="screen",
                parameters=[os.path.join(package, "config", "fleet.yaml")],
            )
        ]
    )
