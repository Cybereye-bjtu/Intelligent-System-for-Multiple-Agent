from glob import glob
from setuptools import find_packages, setup

package_name = 'aaa_search_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='cybereye',
    maintainer_email='devnull@example.com',
    description='Mission-level arbitration for cooperative multi-robot search.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'mission_manager = aaa_search_manager.mission_manager:main',
            'approach_goal_generator = aaa_search_manager.approach_goal_generator:main',
            'target_pose_bridge = aaa_search_manager.target_pose_bridge:main',
            'publish_target_observation = aaa_search_manager.publish_target_observation:main',
            'fake_target_found = aaa_search_manager.fake_target_found:main',
            'fake_frontier_goal = aaa_search_manager.fake_frontier_goal:main',
            'cooperative_wfd = aaa_search_manager.cooperative_wfd:main',
            'semantic_topology = aaa_search_manager.semantic_topology_node:main',
            'target_query_compiler = aaa_search_manager.target_query_compiler:main',
        ],
    },
)
