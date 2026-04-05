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
                "use_yoloe": LaunchConfiguration("vision_use_yoloe"),
                "prompt_classes": LaunchConfiguration("vision_prompt_classes"),
                "published_class_names": LaunchConfiguration(
                    "vision_published_class_names"
                ),
                "conf": LaunchConfiguration("vision_confidence"),
                "color_enable_auto_exposure": LaunchConfiguration(
                    "vision_color_enable_auto_exposure"
                ),
                "color_exposure": LaunchConfiguration("vision_color_exposure"),
                "color_gain": LaunchConfiguration("vision_color_gain"),
                "color_brightness": LaunchConfiguration("vision_color_brightness"),
                "color_contrast": LaunchConfiguration("vision_color_contrast"),
                "color_saturation": LaunchConfiguration("vision_color_saturation"),
                "color_enable_auto_white_balance": LaunchConfiguration(
                    "vision_color_enable_auto_white_balance"
                ),
                "color_white_balance": LaunchConfiguration("vision_color_white_balance"),
                "parent_frame": LaunchConfiguration("vision_parent_frame"),
                "camera_mount_frame": LaunchConfiguration("vision_camera_mount_frame"),
                "camera_optical_frame": LaunchConfiguration("vision_camera_optical_frame"),
                "mount_xyz": LaunchConfiguration("vision_mount_xyz"),
                "mount_rpy_deg": LaunchConfiguration("vision_mount_rpy_deg"),
                "optical_frame_rpy_deg": LaunchConfiguration(
                    "vision_optical_frame_rpy_deg"
                ),
                "grasp_depth_offset_m": LaunchConfiguration(
                    "vision_grasp_depth_offset_m"
                ),
                "publish_image": LaunchConfiguration("vision_publish_image"),
                "show_live_window": LaunchConfiguration("vision_show_live_window"),
                "live_window_name": LaunchConfiguration("vision_live_window_name"),
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
        DeclareLaunchArgument("vision_parent_frame", default_value="vx300s/ee_gripper_link"),
        DeclareLaunchArgument("vision_camera_mount_frame", default_value="camera_mount_frame"),
        DeclareLaunchArgument(
            "vision_camera_optical_frame",
            default_value="camera_color_optical_frame",
        ),
        DeclareLaunchArgument(
            "vision_mount_xyz",
            default_value="-0.0462321,0.0292076,0.0590980",
        ),
        DeclareLaunchArgument(
            "vision_mount_rpy_deg",
            default_value="1.1719,0.6432,1.4275",
        ),
        DeclareLaunchArgument(
            "vision_optical_frame_rpy_deg",
            default_value="-90.0,0.0,-90.0",
        ),
        DeclareLaunchArgument("vision_grasp_depth_offset_m", default_value="0.02"),
        DeclareLaunchArgument("vision_publish_image", default_value="true"),
        DeclareLaunchArgument("vision_show_live_window", default_value="true"),
        DeclareLaunchArgument(
            "vision_live_window_name",
            default_value="VX300 Camera Vision",
        ),
        moveit_launch,
        camera_node,
    ])
