from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robot_manipulation"),
                "launch",
                "vx300_bringup.launch.py",
            ])
        ]),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "robot_name": LaunchConfiguration("robot_name"),
            "motor_port": LaunchConfiguration("motor_port"),
            "use_sim": LaunchConfiguration("use_sim"),
            "load_configs": LaunchConfiguration("load_configs"),
            "xs_driver_logging_level": LaunchConfiguration("xs_driver_logging_level"),
        }.items(),
    )

    diagnostic_node = TimerAction(
        period=LaunchConfiguration("diagnostic_delay"),
        actions=[
            Node(
                package="robot_manipulation",
                executable="vx300_hardware_diagnostics.py",
                name="vx300_hardware_diagnostics",
                arguments=[
                    "--robot-name",
                    LaunchConfiguration("robot_name"),
                    "--arm-group",
                    LaunchConfiguration("arm_group"),
                    "--test-delta",
                    LaunchConfiguration("test_delta"),
                ],
                output="screen",
            )
        ],
        condition=UnlessCondition(LaunchConfiguration("skip_command_test")),
    )

    diagnostic_node_no_motion = TimerAction(
        period=LaunchConfiguration("diagnostic_delay"),
        actions=[
            Node(
                package="robot_manipulation",
                executable="vx300_hardware_diagnostics.py",
                name="vx300_hardware_diagnostics",
                arguments=[
                    "--robot-name",
                    LaunchConfiguration("robot_name"),
                    "--arm-group",
                    LaunchConfiguration("arm_group"),
                    "--test-delta",
                    LaunchConfiguration("test_delta"),
                    "--skip-command-test",
                ],
                output="screen",
            )
        ],
        condition=IfCondition(LaunchConfiguration("skip_command_test")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("load_configs", default_value="false"),
        DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
        DeclareLaunchArgument("arm_group", default_value="arm"),
        DeclareLaunchArgument("test_delta", default_value="0.15"),
        DeclareLaunchArgument("diagnostic_delay", default_value="5.0"),
        DeclareLaunchArgument("skip_command_test", default_value="false"),
        bringup_launch,
        diagnostic_node,
        diagnostic_node_no_motion,
    ])
