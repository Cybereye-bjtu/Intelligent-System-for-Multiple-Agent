#!/usr/bin/env python3
"""Integrate the validated edge-navigation include into the vendor launch."""

import argparse
import os
from pathlib import Path


IMPORT_OLD = (
    "from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, "
    "GroupAction, OpaqueFunction, TimerAction"
)
IMPORT_NEW = """from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction, OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)


def _mqtt_environment(path):
    actions = []
    if not os.path.isfile(path):
        raise RuntimeError(f"MQTT environment file not found: {path}")
    with open(path, encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('export '):
                line = line[7:].strip()
            key, separator, value = line.partition('=')
            if separator and key.strip():
                actions.append(SetEnvironmentVariable(
                    key.strip(), value.strip().strip("'\\\"")
                ))
    return actions"""


def replace_once(text: str, old: str, new: str, description: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"Expected one {description} anchor, found {count}; file was not changed"
        )
    return text.replace(old, new, 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, type=Path)
    parser.add_argument("--robot-name", required=True)
    args = parser.parse_args()

    text = args.file.read_text(encoding="utf-8")
    text = replace_once(text, IMPORT_OLD, IMPORT_NEW, "launch import")

    package_anchor = """    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
"""
    package_block = package_anchor + f"""
    aaa_dir = get_package_share_directory('aaa_navigation')
    aaa_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aaa_dir, 'launch', 'local_path_navigation.launch.py')
        ),
        launch_arguments={{
            'params_file': os.path.join(
                aaa_dir, 'config', '{args.robot_name}_local_navigation.yaml'
            )
        }}.items(),
    )
"""
    text = replace_once(text, package_anchor, package_block, "package path")

    return_old = (
        "    return [sim_arg, master_name_arg, robot_name_arg, "
        "slam_method_arg, bringup_launch]"
    )
    return_new = f"""    return [
        *_mqtt_environment('/home/ubuntu/.{args.robot_name}_mqtt.env'),
        sim_arg, master_name_arg, robot_name_arg, slam_method_arg,
        bringup_launch,
        TimerAction(period=5.0, actions=[aaa_navigation]),
    ]"""
    text = replace_once(text, return_old, return_new, "launch return")

    temporary = args.file.with_suffix(args.file.suffix + ".aaa_tmp")
    temporary.write_text(text, encoding="utf-8")
    os.chmod(temporary, args.file.stat().st_mode)
    os.replace(temporary, args.file)


if __name__ == "__main__":
    main()
