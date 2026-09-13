from glob import glob
import os

from setuptools import find_packages, setup

package_name = "aaa_real_multi_robot"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AAA multi-robot maintainer",
    maintainer_email="maintainer@example.com",
    description="Physical multi-robot slam_toolbox mapping, map fusion, navigation and fleet safety.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "frame_adapter = aaa_real_multi_robot.frame_adapter:main",
            "alignment_manager = aaa_real_multi_robot.alignment_manager:main",
            "map_fuser = aaa_real_multi_robot.map_fuser:main",
            "fleet_coordinator = aaa_real_multi_robot.fleet_coordinator:main",
            "dual_target_coordinator = aaa_real_multi_robot.dual_target_coordinator:main",
            "fleet_safety_manager = aaa_real_multi_robot.fleet_safety_manager:main",
            "lease_gate = aaa_real_multi_robot.lease_gate:main",
            "mqtt_path_sender = aaa_real_multi_robot.mqtt_path_sender:main",
            "system_audit = aaa_real_multi_robot.system_audit:main",
            "robot_visualizer = aaa_real_multi_robot.robot_visualizer:main",
        ]
    },
)
