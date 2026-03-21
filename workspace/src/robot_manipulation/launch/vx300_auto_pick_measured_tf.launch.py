import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    manipulation_share = get_package_share_directory("robot_manipulation")

    auto_pick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manipulation_share, "launch", "vx300_auto_pick.launch.py")
        ),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "robot_name": LaunchConfiguration("robot_name"),
            "motor_port": LaunchConfiguration("motor_port"),
            "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
            "use_viperx_preview": LaunchConfiguration("use_viperx_preview"),
            "xs_driver_logging_level": LaunchConfiguration("xs_driver_logging_level"),
            "auto_pick_config": LaunchConfiguration("auto_pick_config"),
            "launch_camera_vision": LaunchConfiguration("launch_camera_vision"),
            "vision_model_path": LaunchConfiguration("vision_model_path"),
            "vision_confidence": LaunchConfiguration("vision_confidence"),
            "vision_parent_frame": "vx300s/ee_gripper_link",
            "vision_camera_mount_frame": "camera_mount_frame",
            "vision_camera_optical_frame": "camera_color_optical_frame",
            "vision_mount_xyz": "-0.0635,0.0,0.0635",
            "vision_mount_rpy_deg": "0.0,0.0,0.0",
            "vision_optical_frame_rpy_deg": "-90.0,0.0,-90.0",
            "vision_publish_image": LaunchConfiguration("vision_publish_image"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value="vx300s"),
            DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
            DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
            DeclareLaunchArgument("use_viperx_preview", default_value="false"),
            DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
            DeclareLaunchArgument("auto_pick_config", default_value=""),
            DeclareLaunchArgument("launch_camera_vision", default_value="true"),
            DeclareLaunchArgument("vision_model_path", default_value="yolov8n.pt"),
            DeclareLaunchArgument("vision_confidence", default_value="0.50"),
            DeclareLaunchArgument("vision_publish_image", default_value="true"),
            auto_pick_launch,
        ]
    )
