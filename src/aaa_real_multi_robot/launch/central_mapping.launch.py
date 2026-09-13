import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart


def _multirobot_slam_node(package, robot, condition):
    return LifecycleNode(
        package="slam_toolbox",
        executable="decentralized_multirobot_slam_toolbox_node",
        namespace=robot,
        name="slam_toolbox",
        output="screen",
        condition=condition,
        parameters=[
            os.path.join(package, "config", f"{robot}_slam.yaml"),
            {"use_sim_time": False, "use_lifecycle_manager": False},
        ],
        # Keep the existing fleet-wide /tf and /tf_static topics: every frame
        # is already robot-prefixed, so no child has multiple owners.
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
        ],
    )


def _mapping_scan_preprocessor(package, robot, condition):
    return Node(
        package="aaa_navigation",
        executable="scan_preprocessor",
        namespace=robot,
        name="mapping_scan_preprocessor",
        output="screen",
        condition=condition,
        parameters=[
            os.path.join(package, "config", f"{robot}_mapping_scan.yaml"),
            {"use_sim_time": False},
        ],
    )


def _autostart(node, robot, condition):
    return [
        RegisterEventHandler(
            condition=condition,
            event_handler=OnProcessStart(
                target_action=node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    )
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    LogInfo(msg=f"Activating {robot} decentralized slam_toolbox"),
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            ),
            condition=condition,
        ),
    ]


def generate_launch_description():
    package = get_package_share_directory("aaa_real_multi_robot")
    fleet_config = os.path.join(package, "config", "fleet.yaml")
    start_slam = LaunchConfiguration("start_slam")
    start_fleet = LaunchConfiguration("start_fleet_coordinator")
    start_exploration = LaunchConfiguration("start_exploration")
    start_reference_map = LaunchConfiguration("start_reference_map")
    slam_condition = IfCondition(start_slam)
    hyzx_slam = _multirobot_slam_node(package, "hyzx001", slam_condition)
    jetson_slam = _multirobot_slam_node(package, "jetson003", slam_condition)

    actions = [
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_fleet_coordinator", default_value="false"),
        DeclareLaunchArgument("start_exploration", default_value="false"),
        DeclareLaunchArgument("start_reference_map", default_value="false"),
        Node(
            package="aaa_real_multi_robot",
            executable="fleet_safety_manager",
            name="fleet_safety_manager",
            output="screen",
            parameters=[{"default_emergency_stop": True}],
        ),
        _mapping_scan_preprocessor(package, "hyzx001", slam_condition),
        _mapping_scan_preprocessor(package, "jetson003", slam_condition),
        hyzx_slam,
        jetson_slam,
        *_autostart(hyzx_slam, "hyzx001", slam_condition),
        *_autostart(jetson_slam, "jetson003", slam_condition),
        Node(
            package="aaa_real_multi_robot",
            executable="alignment_manager",
            name="alignment_manager",
            output="screen",
            parameters=[fleet_config],
        ),
        TimerAction(
            period=5.0,
            condition=slam_condition,
            actions=[
                Node(
                    package="multirobot_map_merge",
                    executable="map_merge",
                    name="map_merge",
                    output="screen",
                    parameters=[os.path.join(package, "config", "map_merge.yaml")],
                    condition=IfCondition(start_reference_map),
                )
            ],
        ),
        Node(
            package="aaa_real_multi_robot",
            executable="map_fuser",
            name="map_fuser",
            output="screen",
            condition=slam_condition,
            parameters=[fleet_config],
        ),
        Node(
            package="aaa_real_multi_robot",
            executable="fleet_coordinator",
            name="fleet_coordinator",
            output="screen",
            condition=IfCondition(start_fleet),
            parameters=[fleet_config],
        ),
        Node(
            package="aaa_real_multi_robot",
            executable="wfd_explorer",
            name="wfd_explorer",
            output="screen",
            condition=IfCondition(start_exploration),
            parameters=[fleet_config],
        ),
    ]

    return LaunchDescription(actions)
