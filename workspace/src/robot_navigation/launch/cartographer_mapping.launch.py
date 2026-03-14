from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _segment_scan_frame_tfs():
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"lidar_to_segment_scan_frame_static_tf_{index}",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_sensor_tf")),
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "lidar_link",
                "--child-frame-id",
                [LaunchConfiguration("segment_scan_frame_prefix"), f"_{index}"],
            ],
        )
        for index in range(16)
    ]


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
                "sanitized_scan_topic",
                default_value="/scan_segment_sanitized",
                description="LaserScan topic republished for Cartographer after validation",
            ),
            DeclareLaunchArgument(
                "scan_fallback_range_max",
                default_value="10.0",
                description="Fallback LaserScan range_max used when the driver publishes invalid bounds",
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
                "publish_sensor_tf",
                default_value="true",
                description="Publish base->lidar, lidar->lidar_link_1 and lidar->imu static TFs",
            ),
            DeclareLaunchArgument(
                "segment_scan_frame_prefix",
                default_value="lidar_link",
                description="Prefix used by sick_scan_xd segmented LaserScan frame ids",
            ),
            DeclareLaunchArgument("lidar_x", default_value="0.2413"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_x", default_value="0.0124"),
            DeclareLaunchArgument("imu_y", default_value="0.0185"),
            DeclareLaunchArgument("imu_z", default_value="-0.0484"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_static_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_sensor_tf")),
                arguments=[
                    "--x",
                    LaunchConfiguration("lidar_x"),
                    "--y",
                    LaunchConfiguration("lidar_y"),
                    "--z",
                    LaunchConfiguration("lidar_z"),
                    "--roll",
                    LaunchConfiguration("lidar_roll"),
                    "--pitch",
                    LaunchConfiguration("lidar_pitch"),
                    "--yaw",
                    LaunchConfiguration("lidar_yaw"),
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "lidar_link",
                ],
            ),
            *_segment_scan_frame_tfs(),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_to_imu_static_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_sensor_tf")),
                arguments=[
                    "--x",
                    LaunchConfiguration("imu_x"),
                    "--y",
                    LaunchConfiguration("imu_y"),
                    "--z",
                    LaunchConfiguration("imu_z"),
                    "--roll",
                    LaunchConfiguration("imu_roll"),
                    "--pitch",
                    LaunchConfiguration("imu_pitch"),
                    "--yaw",
                    LaunchConfiguration("imu_yaw"),
                    "--frame-id",
                    "lidar_link",
                    "--child-frame-id",
                    "imu_link",
                ],
            ),
            Node(
                package="robot_navigation",
                executable="laser_scan_sanitizer",
                name="cartographer_scan_sanitizer",
                output="screen",
                parameters=[
                    {
                        "fallback_range_max": ParameterValue(
                            LaunchConfiguration("scan_fallback_range_max"), value_type=float
                        )
                    }
                ],
                remappings=[
                    ("scan", LaunchConfiguration("scan_topic")),
                    ("scan_sanitized", LaunchConfiguration("sanitized_scan_topic")),
                ],
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
                    ("scan", LaunchConfiguration("sanitized_scan_topic")),
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
