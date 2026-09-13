from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aaa_search_manager'),
        'config',
        'mission_manager.yaml',
    )
    return LaunchDescription([
        Node(
            package='aaa_search_manager',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='aaa_search_manager',
            executable='target_pose_bridge',
            name='target_pose_bridge',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='aaa_search_manager',
            executable='target_query_compiler',
            name='target_query_compiler',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='aaa_search_manager',
            executable='approach_goal_generator',
            name='approach_goal_generator',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='aaa_search_manager',
            executable='cooperative_wfd',
            name='cooperative_wfd',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='aaa_search_manager',
            executable='semantic_topology',
            name='semantic_topology',
            output='screen',
            parameters=[config],
        ),
    ])
