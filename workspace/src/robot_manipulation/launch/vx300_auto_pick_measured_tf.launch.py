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
            "vision_use_yoloe": LaunchConfiguration("vision_use_yoloe"),
            "vision_prompt_classes": LaunchConfiguration("vision_prompt_classes"),
            "vision_published_class_names": LaunchConfiguration(
                "vision_published_class_names"
            ),
            "vision_confidence": LaunchConfiguration("vision_confidence"),
            "vision_color_enable_auto_exposure": LaunchConfiguration(
                "vision_color_enable_auto_exposure"
            ),
            "vision_color_exposure": LaunchConfiguration("vision_color_exposure"),
            "vision_color_gain": LaunchConfiguration("vision_color_gain"),
            "vision_color_brightness": LaunchConfiguration("vision_color_brightness"),
            "vision_color_contrast": LaunchConfiguration("vision_color_contrast"),
            "vision_color_saturation": LaunchConfiguration("vision_color_saturation"),
            "vision_color_enable_auto_white_balance": LaunchConfiguration(
                "vision_color_enable_auto_white_balance"
            ),
            "vision_color_white_balance": LaunchConfiguration("vision_color_white_balance"),
            "vision_parent_frame": "vx300s/ee_gripper_link",
            "vision_camera_mount_frame": "camera_mount_frame",
            "vision_camera_optical_frame": "camera_color_optical_frame",
            "vision_mount_xyz": "-0.0635,0.0,0.0635",
            "vision_mount_rpy_deg": "0.0,0.0,0.0",
            "vision_optical_frame_rpy_deg": "-90.0,0.0,-90.0",
            "vision_grasp_depth_offset_m": LaunchConfiguration(
                "vision_grasp_depth_offset_m"
            ),
            "vision_publish_image": LaunchConfiguration("vision_publish_image"),
            "vision_show_live_window": LaunchConfiguration("vision_show_live_window"),
            "vision_live_window_name": LaunchConfiguration("vision_live_window_name"),
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
            DeclareLaunchArgument("vision_model_path", default_value="yoloe-11s-seg.pt"),
            DeclareLaunchArgument("vision_use_yoloe", default_value="true"),
            DeclareLaunchArgument(
                "vision_prompt_classes",
                default_value=(
                    "green bottle,green tea bottle,orange bottle,"
                    "clear bottle,"
                    "coca cola can,apple,orange,lemon,"
                    "yellow lays potato chips bag,yellow potato chips bag,"
                    "side of yellow potato chips bag,yellow chips bag side view,"
                    "yellow snack bag"
                ),
            ),
            DeclareLaunchArgument(
                "vision_published_class_names",
                default_value=(
                    "green tea,green tea,roasted tea,"
                    "water,"
                    "can,apple,orange,lemon,"
                    "bag of chips,bag of chips,bag of chips,bag of chips,bag of chips"
                ),
            ),
            DeclareLaunchArgument("vision_confidence", default_value="0.40"),
            DeclareLaunchArgument("vision_color_enable_auto_exposure", default_value="false"),
            DeclareLaunchArgument("vision_color_exposure", default_value="300"),
            DeclareLaunchArgument("vision_color_gain", default_value="24"),
            DeclareLaunchArgument("vision_color_brightness", default_value="0"),
            DeclareLaunchArgument("vision_color_contrast", default_value="50"),
            DeclareLaunchArgument("vision_color_saturation", default_value="64"),
            DeclareLaunchArgument(
                "vision_color_enable_auto_white_balance",
                default_value="true",
            ),
            DeclareLaunchArgument("vision_color_white_balance", default_value="4600"),
            DeclareLaunchArgument("vision_grasp_depth_offset_m", default_value="0.02"),
            DeclareLaunchArgument("vision_publish_image", default_value="true"),
            DeclareLaunchArgument("vision_show_live_window", default_value="true"),
            DeclareLaunchArgument(
                "vision_live_window_name",
                default_value="VX300 Camera Vision",
            ),
            auto_pick_launch,
        ]
    )
