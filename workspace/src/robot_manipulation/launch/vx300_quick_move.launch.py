from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    quick_move_node = TimerAction(
        period=LaunchConfiguration("demo_delay"),
        actions=[
            Node(
                package="robot_manipulation",
                executable="vx300_quick_move.py",
                name="vx300_quick_move",
                arguments=[
                    "--robot-name",
                    LaunchConfiguration("robot_name"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="vx300s"),
        DeclareLaunchArgument("robot_name", default_value=LaunchConfiguration("robot_model")),
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_sim", default_value="false"),
        DeclareLaunchArgument("load_configs", default_value="false"),
        DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
        DeclareLaunchArgument(
            "demo_delay",
            default_value="5.0",
            description="Seconds to wait before starting the quick move node.",
        ),
        bringup_launch,
        quick_move_node,
    ])
