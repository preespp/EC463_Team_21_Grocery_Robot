from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grocery',
    maintainer_email='prees26@bu.edu',
    description='Navigation and Wheel Controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_ros = robot_navigation.teleop_ros:main',
            'wheel_motor = robot_navigation.wheel_motor:main',
            'nav2_serial_bridge = robot_navigation.nav2_serial_bridge:main',
            'base_link_crop_filter = robot_navigation.base_link_crop_filter:main',
            'teleop_cmd_vel = robot_navigation.teleop_cmd_vel:main',
            'teleop_cmd_vel_collision = robot_navigation.teleop_cmd_vel_collision:main',
            'nav_assistant = robot_navigation.nav_assistant:main',
            'semantic_map_server = robot_navigation.semantic_map_server:main',
        ],
    },
)
