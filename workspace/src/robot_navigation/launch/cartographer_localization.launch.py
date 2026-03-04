from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_navigation"))
    default_cfg_dir = str(package_share / "config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "configuration_directory",
                default_value=default_cfg_dir,
                description="Directory containing Cartographer lua config files",
            ),
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="pico_2d_localization.lua",
                description="Cartographer lua localization config file name",
            ),
            DeclareLaunchArgument(
                "load_state_filename",
                default_value="",
                description="Path to Cartographer pbstream map",
            ),
            DeclareLaunchArgument(
                "load_frozen_state",
                default_value="true",
                description="Freeze loaded state for pure localization",
            ),
            DeclareLaunchArgument(
                "points_topic",
                default_value="/cloud_all_fields_fullframe",
                description="PointCloud2 topic from sick_scan_xd",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/sick_scansegment_xd/imu",
                description="IMU topic from sick_scan_xd",
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value="0.03",
                description="Occupancy grid resolution",
            ),
            DeclareLaunchArgument(
                "publish_period_sec",
                default_value="1.0",
                description="Occupancy grid publish period",
            ),
            DeclareLaunchArgument(
                "publish_occupancy_grid",
                default_value="false",
                description="Whether to publish Cartographer occupancy grid during localization",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                arguments=[
                    "-configuration_directory",
                    LaunchConfiguration("configuration_directory"),
                    "-configuration_basename",
                    LaunchConfiguration("configuration_basename"),
                    "-load_state_filename",
                    LaunchConfiguration("load_state_filename"),
                    "-load_frozen_state",
                    LaunchConfiguration("load_frozen_state"),
                ],
                remappings=[
                    ("points2", LaunchConfiguration("points_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="carto_grid",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_occupancy_grid")),
                parameters=[
                    {
                        "resolution": ParameterValue(
                            LaunchConfiguration("resolution"), value_type=float
                        ),
                        "publish_period_sec": ParameterValue(
                            LaunchConfiguration("publish_period_sec"), value_type=float
                        ),
                    }
                ],
                remappings=[("/map", "/cartographer_map")],
            ),
        ]
    )
