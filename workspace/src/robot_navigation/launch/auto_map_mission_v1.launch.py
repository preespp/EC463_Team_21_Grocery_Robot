from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in [current.parent, *current.parents]:
        if (parent / "Nav").exists() and (parent / "Maps").exists() and (parent / "workspace").exists():
            return parent
    cwd = Path.cwd()
    if (cwd / "Nav").exists() and (cwd / "Maps").exists():
        return cwd
    return cwd


def generate_launch_description():
    repo_root = _find_repo_root()
    package_share = Path(get_package_share_directory("robot_navigation"))
    maps_dir = repo_root / "Maps"

    default_nav2_params = str(package_share / "config" / "nav2_params_cartographer_mapping.yaml")
    default_ekf_params = str(package_share / "config" / "ekf_odom_base_imu.yaml")
    default_maps_dir = str(maps_dir)

    slam_mapping_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_navigation"), "launch", "slam_mapping_stack.launch.py"]
            )
        ),
        launch_arguments={
            "hostname": LaunchConfiguration("hostname"),
            "udp_receiver_ip": LaunchConfiguration("udp_receiver_ip"),
            "serial_port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "cmd_topics": LaunchConfiguration("cmd_topics"),
            "manual_cmd_topic": LaunchConfiguration("manual_cmd_topic"),
            "output_cmd_topic": LaunchConfiguration("output_cmd_topic"),
            "manual_override_topic": LaunchConfiguration("manual_override_topic"),
            "manual_cmd_timeout": LaunchConfiguration("manual_cmd_timeout"),
            "auto_cmd_timeout": LaunchConfiguration("auto_cmd_timeout"),
            "arbiter_publish_rate": LaunchConfiguration("arbiter_publish_rate"),
            "arbiter_stop_on_source_switch": LaunchConfiguration("arbiter_stop_on_source_switch"),
            "telemetry_enabled": LaunchConfiguration("telemetry_enabled"),
            "left_switch": LaunchConfiguration("left_switch"),
            "right_switch": LaunchConfiguration("right_switch"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "fallback_odom": LaunchConfiguration("fallback_odom"),
            "use_ekf": LaunchConfiguration("use_ekf"),
            "ekf_params_file": LaunchConfiguration("ekf_params_file"),
            "cartographer_config_basename": LaunchConfiguration("cartographer_config_basename"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "sick_tf_publish_rate": LaunchConfiguration("sick_tf_publish_rate"),
            "imu_udp_port": LaunchConfiguration("imu_udp_port"),
            "scandataformat": LaunchConfiguration("scandataformat"),
            "send_sopas_start_stop_cmd": LaunchConfiguration("send_sopas_start_stop_cmd"),
            "host_set_frecho_filter": LaunchConfiguration("host_set_frecho_filter"),
            "host_set_lfp_angle_range_filter": LaunchConfiguration("host_set_lfp_angle_range_filter"),
            "host_set_lfp_interval_filter": LaunchConfiguration("host_set_lfp_interval_filter"),
            "lidar_x": LaunchConfiguration("lidar_x"),
            "lidar_y": LaunchConfiguration("lidar_y"),
            "lidar_z": LaunchConfiguration("lidar_z"),
            "lidar_roll": LaunchConfiguration("lidar_roll"),
            "lidar_pitch": LaunchConfiguration("lidar_pitch"),
            "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            "with_collision": LaunchConfiguration("with_collision"),
            "with_rviz": LaunchConfiguration("with_slam_rviz"),
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("nav2_namespace"),
            "use_sim_time": "false",
            "autostart": "true",
            "params_file": LaunchConfiguration("nav2_params_file"),
        }.items(),
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "rviz_launch.py"]
            )
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("nav2_namespace"),
            "use_namespace": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("with_nav2_rviz")),
    )

    mission_orchestrator = Node(
        package="robot_navigation",
        executable="mission_orchestrator",
        output="screen",
        parameters=[
            {
                "autostart": ParameterValue(LaunchConfiguration("autostart_mission"), value_type=bool),
                "nav2_namespace": LaunchConfiguration("nav2_namespace"),
                "manual_override_topic": LaunchConfiguration("manual_override_topic"),
                "odom_topic": "/odom",
                "boot_capture_delay_sec": ParameterValue(
                    LaunchConfiguration("boot_capture_delay_sec"), value_type=float
                ),
                "mapping_timeout_sec": ParameterValue(
                    LaunchConfiguration("mapping_timeout_sec"), value_type=float
                ),
                "mapping_max_distance_m": ParameterValue(
                    LaunchConfiguration("mapping_max_distance_m"), value_type=float
                ),
                "home_retry_limit": ParameterValue(
                    LaunchConfiguration("home_retry_limit"), value_type=int
                ),
                "map_output_dir": LaunchConfiguration("map_output_dir"),
                "map_name": LaunchConfiguration("map_name"),
                "map_name_prefix": LaunchConfiguration("map_name_prefix"),
                "export_map": ParameterValue(LaunchConfiguration("export_map"), value_type=bool),
                "export_resolution": ParameterValue(
                    LaunchConfiguration("export_resolution"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hostname", default_value="192.168.8.150"),
            DeclareLaunchArgument("udp_receiver_ip", default_value="192.168.8.249"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument(
                "cmd_topics",
                default_value='["/cmd_vel_auto","/cmd_vel_nav","/cmd_vel_smoothed"]',
            ),
            DeclareLaunchArgument("manual_cmd_topic", default_value="/cmd_vel_manual"),
            DeclareLaunchArgument("output_cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("manual_override_topic", default_value="/manual_override"),
            DeclareLaunchArgument("manual_cmd_timeout", default_value="0.35"),
            DeclareLaunchArgument("auto_cmd_timeout", default_value="0.35"),
            DeclareLaunchArgument("arbiter_publish_rate", default_value="50.0"),
            DeclareLaunchArgument("arbiter_stop_on_source_switch", default_value="true"),
            DeclareLaunchArgument("telemetry_enabled", default_value="true"),
            DeclareLaunchArgument("left_switch", default_value="1"),
            DeclareLaunchArgument("right_switch", default_value="1"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_raw"),
            DeclareLaunchArgument("fallback_odom", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            DeclareLaunchArgument("cartographer_config_basename", default_value="pico_2d.lua"),
            DeclareLaunchArgument("imu_topic", default_value="/sick_scansegment_xd/imu"),
            DeclareLaunchArgument("sick_tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("imu_udp_port", default_value="7503"),
            DeclareLaunchArgument("scandataformat", default_value="2"),
            DeclareLaunchArgument("send_sopas_start_stop_cmd", default_value="0"),
            DeclareLaunchArgument("host_set_frecho_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_angle_range_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_interval_filter", default_value="0"),
            DeclareLaunchArgument("lidar_x", default_value="0.254"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("with_collision", default_value="true"),
            DeclareLaunchArgument("with_slam_rviz", default_value="false"),
            DeclareLaunchArgument("with_nav2_rviz", default_value="false"),
            DeclareLaunchArgument("nav2_namespace", default_value=""),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("autostart_mission", default_value="true"),
            DeclareLaunchArgument("boot_capture_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("mapping_timeout_sec", default_value="180.0"),
            DeclareLaunchArgument("mapping_max_distance_m", default_value="80.0"),
            DeclareLaunchArgument("home_retry_limit", default_value="2"),
            DeclareLaunchArgument("map_output_dir", default_value=default_maps_dir),
            DeclareLaunchArgument("map_name", default_value=""),
            DeclareLaunchArgument("map_name_prefix", default_value="run"),
            DeclareLaunchArgument("export_map", default_value="true"),
            DeclareLaunchArgument("export_resolution", default_value="0.03"),
            slam_mapping_stack,
            TimerAction(period=2.0, actions=[nav2_bringup]),
            mission_orchestrator,
            nav2_rviz,
        ]
    )
