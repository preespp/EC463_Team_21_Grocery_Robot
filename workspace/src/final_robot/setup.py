from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'final_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grocery',
    maintainer_email='prees26@bu.edu',
    description='Final Robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'lidar.py = final_robot.lidar:main',
            # 'ir_sensor = final_robot.ir_sensor:main',
            'navigation = final_robot.navigation:main',
            'wheel_motor = final_robot.wheel_motor:main',
            'distance_sensor = final_robot.distance_sensor:main',
            'camera_vision = final_robot.camera_vision:main',
            # 'arm_motor = final_robot.arm_motor_I2C:main',
            'arm_controller = final_robot.arm_controller:main',
            'arm_to_gazebo = final_robot.arm_to_gazebo:main',
            'ui_input = final_robot.ui_input:main',
            'central = final_robot.central:main',
        ],
    },
)
