import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
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
    use_sim = LaunchConfiguration("use_sim").perform(context).lower() == "true"
    load_configs = LaunchConfiguration("load_configs").perform(context).lower() == "true"
    driver_logging_level = LaunchConfiguration("xs_driver_logging_level").perform(context)

    if robot_model not in {"vx300", "vx300s"}:
        raise RuntimeError("robot_model must be 'vx300' or 'vx300s'.")

    try:
        get_package_share_directory("interbotix_xs_sdk")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "interbotix_xs_sdk is not available. Source "
            "'/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash' "
            "before launching robot_manipulation."
        ) from exc

    pkg_dir = get_package_share_directory("robot_manipulation")
    motor_configs = os.path.join(pkg_dir, "config", f"{robot_model}.yaml")
    mode_template = resolve_model_config_path(
        pkg_dir,
        robot_model,
        "_xsarm_modes.yaml",
        "vx300_xsarm_modes.yaml",
    )
    urdf_file = os.path.join(pkg_dir, "urdf", "interbotix_xsarm", f"{robot_model}.urdf.xacro")

    with open(mode_template, "r", encoding="utf-8") as template_file:
        mode_config_contents = template_file.read().replace("__MOTOR_PORT__", motor_port)

    generated_mode_config = os.path.join(
        tempfile.gettempdir(),
        f"{robot_model}_{robot_name}_modes.yaml",
    )
    with open(generated_mode_config, "w", encoding="utf-8") as mode_file:
        mode_file.write(mode_config_contents)

    hardware_type = "fake" if use_sim else "actual"
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "hardware_type:=",
            hardware_type,
        ]),
        value_type=str,
    )

    xs_sdk_executable = "xs_sdk_sim.py" if use_sim else "xs_sdk"
    xs_sdk_node_name = "xs_sdk_sim" if use_sim else "xs_sdk"

    return [
        Node(
            package="interbotix_xs_sdk",
            executable=xs_sdk_executable,
            namespace=robot_name,
            name=xs_sdk_node_name,
            parameters=[{
                "motor_configs": motor_configs,
                "mode_configs": generated_mode_config,
                "load_configs": load_configs,
                "robot_description": robot_description,
                "xs_driver_logging_level": driver_logging_level,
            }],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=f"{robot_name}_state_publisher",
            parameters=[{"robot_description": robot_description}],
            remappings=[
                ("joint_states", f"/{robot_name}/joint_states"),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_model",
            default_value="vx300s",
            description="Interbotix robot model. Supported values: vx300, vx300s.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value=LaunchConfiguration("robot_model"),
            description="ROS namespace and name prefix used by the VX300 bringup.",
        ),
        DeclareLaunchArgument(
            "motor_port",
            default_value=ARM_MOTOR_PORT,
            description="Serial port for the Dynamixel adapter when using real hardware.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Use interbotix_xs_sdk simulator instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "load_configs",
            default_value="false",
            description="Whether xs_sdk should write EEPROM config values on startup.",
        ),
        DeclareLaunchArgument(
            "xs_driver_logging_level",
            default_value="INFO",
            description="Logging level for the Interbotix X-Series driver.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
