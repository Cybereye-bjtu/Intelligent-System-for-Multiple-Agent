from glob import glob
import os

from setuptools import find_packages, setup


package_name = "aaa_navigation"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy", "paho-mqtt"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="AAA ROS 2 maintainer",
    maintainer_email="maintainer@example.com",
    description="Self-contained ROS 2 cloud navigation for the HYZX and Jetson003 fleet.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_preprocessor = aaa_navigation.scan_preprocessor:main",
            "esdf_mapper = aaa_navigation.esdf_mapper:main",
            "astar_planner = aaa_navigation.astar_planner:main",
            "vfh_controller = aaa_navigation.vfh_controller:main",
            "safety_barrier = aaa_navigation.safety_barrier:main",
            "cmd_vel_gateway = aaa_navigation.cmd_vel_gateway:main",
            "cloud_command_sender = aaa_navigation.cloud_command_sender:main",
            "edge_command_receiver = aaa_navigation.edge_command_receiver:main",
            "mqtt_cloud_bridge = aaa_navigation.mqtt_cloud_bridge:main",
            "mqtt_edge_bridge = aaa_navigation.mqtt_edge_bridge:main",
            "mqtt_path_receiver = aaa_navigation.mqtt_path_receiver:main",
            "kettle_navigator = aaa_navigation.kettle_navigator:main",
        ],
    },
)
