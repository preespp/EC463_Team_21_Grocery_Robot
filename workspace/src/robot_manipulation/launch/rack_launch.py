from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory("robot_manipulation")
    param_file = os.path.join(pkg_dir, "config", "rack_position.yaml")

    return LaunchDescription([
        Node(
            package="robot_manipulation",
            executable="rack_controller",
            name="rack_controller",
            parameters=[param_file]
        )
    ])
