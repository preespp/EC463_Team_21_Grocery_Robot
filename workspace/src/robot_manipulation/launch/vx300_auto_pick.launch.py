import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    manipulation_share = get_package_share_directory("robot_manipulation")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulation_share, "launch", "vx300_moveit.launch.py")
        ),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "robot_name": LaunchConfiguration("robot_name"),
            "motor_port": LaunchConfiguration("motor_port"),
            "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
            "use_viperx_arm_server": "true",
            "use_viperx_preview": LaunchConfiguration("use_viperx_preview"),
            "use_auto_pick": "true",
            "auto_pick_config": LaunchConfiguration("auto_pick_config"),
            "xs_driver_logging_level": LaunchConfiguration("xs_driver_logging_level"),
        }.items(),
    )

    camera_node = Node(
        package="robot_vision",
        executable="camera_vision",
        name="camera_vision",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("vision_model_path"),
                "conf": LaunchConfiguration("vision_confidence"),
                "parent_frame": LaunchConfiguration("vision_parent_frame"),
                "camera_mount_frame": LaunchConfiguration("vision_camera_mount_frame"),
                "camera_optical_frame": LaunchConfiguration("vision_camera_optical_frame"),
                "mount_xyz": LaunchConfiguration("vision_mount_xyz"),
                "mount_rpy_deg": LaunchConfiguration("vision_mount_rpy_deg"),
                "optical_frame_rpy_deg": LaunchConfiguration(
                    "vision_optical_frame_rpy_deg"
                ),
                "publish_image": LaunchConfiguration("vision_publish_image"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_camera_vision")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300s"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument("use_viperx_preview", default_value="false"),
        DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
        DeclareLaunchArgument("auto_pick_config", default_value=""),
        DeclareLaunchArgument("launch_camera_vision", default_value="false"),
        DeclareLaunchArgument("vision_model_path", default_value="yolov8n.pt"),
        DeclareLaunchArgument("vision_confidence", default_value="0.50"),
        DeclareLaunchArgument("vision_parent_frame", default_value="vx300s/ee_gripper_link"),
        DeclareLaunchArgument("vision_camera_mount_frame", default_value="camera_mount_frame"),
        DeclareLaunchArgument(
            "vision_camera_optical_frame",
            default_value="camera_color_optical_frame",
        ),
        DeclareLaunchArgument(
            "vision_mount_xyz",
            default_value="-0.98129,-0.256618,-0.0501378",
        ),
        DeclareLaunchArgument(
            "vision_mount_rpy_deg",
            default_value="-0.4593975,0.2464277,8.1401713",
        ),
        DeclareLaunchArgument(
            "vision_optical_frame_rpy_deg",
            default_value="-90.0,0.0,-90.0",
        ),
        DeclareLaunchArgument("vision_publish_image", default_value="true"),
        moveit_launch,
        camera_node,
    ])
