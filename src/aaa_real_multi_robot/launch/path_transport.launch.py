from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot_name").perform(context).strip("/")
    robot_id = int(LaunchConfiguration("robot_id").perform(context))
    upstream_params = LaunchConfiguration("upstream_params").perform(context)
    with open(upstream_params, encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    mqtt = dict(document.get("mqtt_cloud_bridge", {}).get("ros__parameters", {}))
    allowed = {
        "broker", "port", "keepalive_seconds", "username_env", "password_env",
        "ca_file", "tls_insecure",
    }
    mqtt = {key: value for key, value in mqtt.items() if key in allowed}
    mqtt.update({
        "client_id": f"aaa-{robot}-path-cloud",
        "path_mqtt_topic": f"edge/{robot}/path",
        "path_ros_topic": f"/{robot}/global_path",
        "lease_topic": f"/{robot}/fleet/lease",
        "robot_id": robot_id,
        "global_frame": "site_map",
        "target_frame": f"{robot}/odom",
        "output_frame": "odom",
        "enabled_on_start": False,
    })
    return [Node(
        package="aaa_real_multi_robot",
        executable="mqtt_path_sender",
        namespace=robot,
        name="mqtt_path_sender",
        output="screen",
        parameters=[mqtt, {"use_sim_time": False}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name"),
        DeclareLaunchArgument("robot_id"),
        DeclareLaunchArgument(
            "upstream_params",
            default_value=(
                get_package_share_directory("aaa_navigation")
                + "/config/jetson003_mqtt_cloud.yaml"
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
