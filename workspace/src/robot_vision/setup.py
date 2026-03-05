from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='grocery',
    maintainer_email='prees26@bu.edu',
    description='Camera & Vision Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_vision = robot_vision.camera_vision:main',
            'bar_code = robot_vision.bar_code:main',
            'realsense_combination = robot_vision.realsense_combination_node:main',
        ],
    },
)
