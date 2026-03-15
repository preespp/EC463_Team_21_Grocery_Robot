from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_manipulation")
    viperx_server_param = os.path.join(pkg_dir, "config", "viperx_arm_server.yaml")

    return LaunchDescription([
        Node(
            package="robot_manipulation",
            executable="viperx_arm_server",
            name="viperx_arm_server",
            output="screen",
            parameters=[viperx_server_param],
        ),
    ])
