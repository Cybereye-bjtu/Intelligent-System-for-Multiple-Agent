import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot_name").perform(context).strip("/")
    config = LaunchConfiguration("config_file").perform(context)
    return [
        Node(
            package="aaa_real_multi_robot",
            executable="frame_adapter",
            namespace=robot,
            name="frame_adapter",
            output="screen",
            parameters=[config, {"robot_name": robot, "use_sim_time": False}],
        )
    ]


def generate_launch_description():
    package = get_package_share_directory("aaa_real_multi_robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="hyzx001"),
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(package, "config", "hyzx001_adapter.yaml"),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

