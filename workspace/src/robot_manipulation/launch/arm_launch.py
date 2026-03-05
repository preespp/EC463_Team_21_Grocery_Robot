from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("robot_manipulation")
    arm_controller_param = os.path.join(pkg_dir, "config", "arm_controller.yaml")
    arm_motor_param = os.path.join(pkg_dir, "config", "arm_motor.yaml")

    return LaunchDescription([
        Node(
            package="robot_manipulation",
            executable="arm_controller",
            name="arm_controller",
            output="screen",
            parameters=[arm_controller_param],
        ),
        Node(
            package="robot_manipulation",
            executable="arm_motor",
            name="arm_motor",
            output="screen",
            parameters=[arm_motor_param],
        ),
    ])
