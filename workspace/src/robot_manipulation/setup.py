from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_manipulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grocery',
    maintainer_email='prees26@bu.edu',
    description='Robotics Arm and Racking System',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rack_controller = robot_manipulation.rack_controller:main',
            'arm_motor = robot_manipulation.arm_motor:main',
            'arm_controller = robot_manipulation.arm_controller:main',
            'arm_to_gazebo = robot_manipulation.arm_to_gazebo:main',
        ],
    },
)
