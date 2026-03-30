import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    pkg_dir = get_package_share_directory("robot_manipulation")
    moveit_pkg = get_package_share_directory("interbotix_xsarm_moveit")
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    preview_only = LaunchConfiguration("preview_only").perform(context).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    viperx_server_param = os.path.join(pkg_dir, "config", "viperx_arm_server.yaml")
    urdf_file = os.path.join(pkg_dir, "urdf", "interbotix_xsarm", f"{robot_model}.urdf.xacro")
    srdf_file = os.path.join(moveit_pkg, "config", "srdf", f"{robot_model}.srdf.xacro")

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
        Node(
            package="robot_manipulation",
            executable="viperx_arm_server",
            name="viperx_arm_server",
            output="screen",
            remappings=[
                ("joint_states", f"/{robot_name}/joint_states"),
            ],
            parameters=[
                viperx_server_param,
                kinematics_config,
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "preview_only": preview_only,
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300s"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("preview_only", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
