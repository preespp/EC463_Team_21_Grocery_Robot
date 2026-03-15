import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    motor_port = LaunchConfiguration("motor_port").perform(context)
    use_moveit_rviz = LaunchConfiguration("use_moveit_rviz").perform(context)
    xs_driver_logging_level = LaunchConfiguration("xs_driver_logging_level").perform(context)

    if robot_model not in {"vx300", "vx300s"}:
        raise RuntimeError("robot_model must be 'vx300' or 'vx300s'.")

    try:
        moveit_pkg = get_package_share_directory("interbotix_xsarm_moveit")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "interbotix_xsarm_moveit is not available. Build and source "
            "'/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws' "
            "before launching MoveIt."
        ) from exc

    pkg_dir = get_package_share_directory("robot_manipulation")
    mode_template = os.path.join(pkg_dir, "config", "vx300_moveit_modes.yaml")

    with open(mode_template, "r", encoding="utf-8") as template_file:
        mode_config_contents = template_file.read().replace("__MOTOR_PORT__", motor_port)

    generated_mode_config = os.path.join(
        tempfile.gettempdir(),
        f"{robot_model}_{robot_name}_moveit_modes.yaml",
    )
    with open(generated_mode_config, "w", encoding="utf-8") as mode_file:
        mode_file.write(mode_config_contents)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(moveit_pkg, "launch", "xsarm_moveit.launch.py")
            ]),
            launch_arguments={
                "robot_model": robot_model,
                "robot_name": robot_name,
                "mode_configs": generated_mode_config,
                "xs_driver_logging_level": xs_driver_logging_level,
                "use_moveit_rviz": use_moveit_rviz,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
        OpaqueFunction(function=launch_setup),
    ])
