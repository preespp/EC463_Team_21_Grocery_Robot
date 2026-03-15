from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    run_arm_controller = LaunchConfiguration("run_arm_controller")

    pkg_dir = get_package_share_directory("robot_manipulation")
    arm_controller_param = os.path.join(pkg_dir, "config", "arm_controller.yaml")
    arm_to_gazebo_param = os.path.join(pkg_dir, "config", "arm_to_gazebo.yaml")
    bridge_config = os.path.join(pkg_dir, "config", "arm_bridge.yaml")
    urdf_file = os.path.join(pkg_dir, "urdf", "five_dof_arm.urdf")
    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            "run_arm_controller",
            default_value="false",
            description="Start arm_controller (requires MoveIt config with SRDF running).",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="robot_manipulation",
            executable="arm_controller",
            name="arm_controller",
            output="screen",
            parameters=[arm_controller_param, {"robot_description": robot_description}],
            condition=IfCondition(run_arm_controller),
        ),
        Node(
            package="robot_manipulation",
            executable="arm_to_gazebo",
            name="arm_to_gazebo",
            output="screen",
            parameters=[arm_to_gazebo_param],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="arm_cmd_bridge",
            output="screen",
            parameters=[{"config_file": bridge_config}],
        ),
    ])
