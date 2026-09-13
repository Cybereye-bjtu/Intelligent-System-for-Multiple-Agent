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
                executable="robot_visualizer",
                name="robot_visualizer",
                output="screen",
                parameters=[{"use_sim_time": False, "safety_radius": 0.35}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="aaa_real_multi_robot_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(package, "rviz", "aaa_real_multi_robot.rviz"),
                ],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
