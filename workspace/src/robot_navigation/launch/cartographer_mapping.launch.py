from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
                default_value="pico_2d_mapping_quality_scan_segment.lua",
                description="Cartographer lua configuration file name",
            ),
            DeclareLaunchArgument(
                "points_topic",
                default_value="/cloud_all_fields_fullframe",
                description="PointCloud2 topic from sick_scan_xd",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan_segment",
                description="LaserScan topic from sick_scan_xd",
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
                ],
                remappings=[
                    ("scan", LaunchConfiguration("scan_topic")),
                    ("points2", LaunchConfiguration("points_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="carto_grid",
                output="screen",
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
            ),
        ]
    )
