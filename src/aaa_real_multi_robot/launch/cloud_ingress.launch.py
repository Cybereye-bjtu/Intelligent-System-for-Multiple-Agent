from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot_name").perform(context).strip("/")
    upstream_params = LaunchConfiguration("upstream_params").perform(context)
    adapter_params = LaunchConfiguration("adapter_params").perform(context)
    ingress_prefix = f"/ingress/{robot}"
    with open(upstream_params, encoding="utf-8") as stream:
        upstream_document = yaml.safe_load(stream) or {}
    bridge_params = dict(
        upstream_document.get("mqtt_cloud_bridge", {}).get("ros__parameters", {})
    )
    if not bridge_params.get("broker"):
        raise RuntimeError(f"mqtt_cloud_bridge parameters missing from {upstream_params}")
    return [
        Node(
            package="aaa_navigation",
            executable="mqtt_cloud_bridge",
            namespace=f"{robot}_ingress",
            name="mqtt_cloud_bridge",
            output="screen",
            parameters=[
                bridge_params,
                {
                    "use_sim_time": False,
                    "scan_ros_topic": f"{ingress_prefix}/scan",
                    "odom_ros_topic": f"{ingress_prefix}/odom",
                    "robot_status_ros_topic": f"{ingress_prefix}/status",
                    "command_ros_topic": f"/{robot}/cmd_vel_cloud",
                    "publish_command_control": True,
                    "publish_odom_tf": False,
                    "publish_configured_laser_tf": False,
                    "publish_configured_odom_link_tf": False,
                    "tf_mqtt_topic": f"disabled/{robot}/tf",
                    "tf_static_mqtt_topic": f"disabled/{robot}/tf_static",
                    # Central receipt time avoids depending on vehicle clocks.
                    "use_envelope_timestamp": False,
                },
            ],
        ),
        Node(
            package="aaa_real_multi_robot",
            executable="frame_adapter",
            namespace=robot,
            name="frame_adapter",
            output="screen",
            parameters=[
                adapter_params,
                {
                    "robot_name": robot,
                    "input_scan_topic": f"{ingress_prefix}/scan",
                    "input_odom_topic": f"{ingress_prefix}/odom",
                    "use_sim_time": False,
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name"),
            DeclareLaunchArgument("upstream_params"),
            DeclareLaunchArgument("adapter_params"),
            OpaqueFunction(function=launch_setup),
        ]
    )
