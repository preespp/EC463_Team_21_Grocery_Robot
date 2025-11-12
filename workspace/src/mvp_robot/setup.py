from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mvp_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grocery',
    maintainer_email='prees26@bu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'product_db_query = mvp_robot.product_db_query:main',
            # 'lidar.py = mvp_robot.lidar:main',
            # 'ir_sensor = mvp_robot.ir_sensor:main',
            # 'navigation = mvp_robot.navigation:main',
            # 'wheel_motor = mvp_robot.wheel_motor:main',
            'distance_sensor = mvp_robot.distance_sensor:main',
            # 'camera_vision = mvp_robot.camera_vision:main',
            # 'arm_motor = mvp_robot.arm_motor:main',
            'ui_input = mvp_robot.ui_input:main',
            'central = mvp_robot.central:main',
        ],
    },
)
