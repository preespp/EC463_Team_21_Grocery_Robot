import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


ARM_MOTOR_PORT = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0"
)


def resolve_model_config_path(pkg_dir: str, robot_model: str, suffix: str, fallback_name: str) -> str:
    model_specific = os.path.join(pkg_dir, "config", f"{robot_model}{suffix}")
    if os.path.exists(model_specific):
        return model_specific
    return os.path.join(pkg_dir, "config", fallback_name)


def launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    motor_port = LaunchConfiguration("motor_port").perform(context)
    use_moveit_rviz = LaunchConfiguration("use_moveit_rviz").perform(context)
    xs_driver_logging_level = LaunchConfiguration("xs_driver_logging_level").perform(context)
    use_viperx_arm_server = LaunchConfiguration("use_viperx_arm_server").perform(context)
    use_viperx_preview = LaunchConfiguration("use_viperx_preview").perform(context)
    use_auto_pick = LaunchConfiguration("use_auto_pick").perform(context)
    auto_pick_config = LaunchConfiguration("auto_pick_config").perform(context)
    preview_only = use_viperx_preview.lower() in {"1", "true", "yes", "on"}

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
    if not auto_pick_config:
        auto_pick_config = resolve_model_config_path(
            pkg_dir,
            robot_model,
            "_auto_pick.yaml",
            "vx300_auto_pick.yaml",
        )
    mode_template = resolve_model_config_path(
        pkg_dir,
        robot_model,
        "_moveit_modes.yaml",
        "vx300_moveit_modes.yaml",
    )
    urdf_file = os.path.join(pkg_dir, "urdf", "interbotix_xsarm", f"{robot_model}.urdf.xacro")
    srdf_file = os.path.join(moveit_pkg, "config", "srdf", f"{robot_model}.srdf.xacro")

    with open(mode_template, "r", encoding="utf-8") as template_file:
        mode_config_contents = template_file.read().replace("__MOTOR_PORT__", motor_port)

    generated_mode_config = os.path.join(
        tempfile.gettempdir(),
        f"{robot_model}_{robot_name}_moveit_modes.yaml",
    )
    with open(generated_mode_config, "w", encoding="utf-8") as mode_file:
        mode_file.write(mode_config_contents)

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "hardware_type:=actual",
        ]),
        value_type=str,
    )
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            srdf_file,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "base_link_frame:=base_link",
            " ",
            "show_ar_tag:=false",
            " ",
            "external_srdf_loc:=",
            "",
        ]),
        value_type=str,
    )
    kinematics_config = os.path.join(moveit_pkg, "config", "kinematics.yaml")

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
        ),
        Node(
            package="robot_manipulation",
            executable="viperx_arm_server",
            name="viperx_arm_server",
            output="screen",
            remappings=[
                ("joint_states", f"/{robot_name}/joint_states"),
            ],
            parameters=[
                os.path.join(pkg_dir, "config", "viperx_arm_server.yaml"),
                kinematics_config,
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "preview_only": preview_only,
                },
            ],
            condition=IfCondition(use_viperx_arm_server),
        ),
        Node(
            package="robot_manipulation",
            executable="vision_auto_pick.py",
            name="vision_auto_pick",
            output="screen",
            parameters=[
                auto_pick_config,
                {
                    "base_frame": f"{robot_name}/base_link",
                    "ee_orientation_frame": f"{robot_name}/ee_gripper_link",
                },
            ],
            condition=IfCondition(use_auto_pick),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300s"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("motor_port", default_value=ARM_MOTOR_PORT),
        DeclareLaunchArgument("use_moveit_rviz", default_value="false"),
        DeclareLaunchArgument("use_viperx_arm_server", default_value="true"),
        DeclareLaunchArgument("use_viperx_preview", default_value="false"),
        DeclareLaunchArgument("use_auto_pick", default_value="false"),
        DeclareLaunchArgument(
            "auto_pick_config",
            default_value="",
        ),
        DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
        OpaqueFunction(function=launch_setup),
    ])
