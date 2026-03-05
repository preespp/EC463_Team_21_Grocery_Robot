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

    default_nav2_params = str(package_share / "config" / "nav2_params_explore_slow.yaml")
    default_ekf_params = str(package_share / "config" / "ekf_odom_base_imu.yaml")
    default_shelves_file = str(package_share / "config" / "shelves.yaml")
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

    frontier_explorer = Node(
        package="robot_navigation",
        executable="frontier_explorer",
        output="screen",
        parameters=[
            {
                "autostart": ParameterValue(LaunchConfiguration("autostart_frontier"), value_type=bool),
                "map_topic": LaunchConfiguration("map_topic"),
                "manual_override_topic": LaunchConfiguration("manual_override_topic"),
                "nav2_namespace": LaunchConfiguration("nav2_namespace"),
                "navigate_to_pose_action": LaunchConfiguration("navigate_to_pose_action"),
                "global_frame": LaunchConfiguration("global_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "state_topic": LaunchConfiguration("frontier_state_topic"),
                "done_topic": LaunchConfiguration("frontier_done_topic"),
                "goal_topic": LaunchConfiguration("frontier_goal_topic"),
                "start_service": LaunchConfiguration("frontier_start_service"),
                "stop_service": LaunchConfiguration("frontier_stop_service"),
                "loop_rate_hz": ParameterValue(LaunchConfiguration("frontier_loop_rate_hz"), value_type=float),
                "min_frontier_cluster_size": ParameterValue(
                    LaunchConfiguration("min_frontier_cluster_size"), value_type=int
                ),
                "goal_timeout_sec": ParameterValue(LaunchConfiguration("frontier_goal_timeout_sec"), value_type=float),
                "blacklist_ttl_sec": ParameterValue(LaunchConfiguration("frontier_blacklist_ttl_sec"), value_type=float),
                "blacklist_radius_m": ParameterValue(LaunchConfiguration("frontier_blacklist_radius_m"), value_type=float),
                "no_frontier_rounds_limit": ParameterValue(
                    LaunchConfiguration("frontier_no_frontier_rounds_limit"), value_type=int
                ),
                "max_explore_time_sec": ParameterValue(LaunchConfiguration("max_explore_time_sec"), value_type=float),
                "new_area_ratio_threshold": ParameterValue(
                    LaunchConfiguration("frontier_new_area_ratio_threshold"), value_type=float
                ),
                "new_area_window_sec": ParameterValue(
                    LaunchConfiguration("frontier_new_area_window_sec"), value_type=float
                ),
                "score_w_gain": ParameterValue(LaunchConfiguration("frontier_score_w_gain"), value_type=float),
                "score_w_path": ParameterValue(LaunchConfiguration("frontier_score_w_path"), value_type=float),
                "score_w_risk": ParameterValue(LaunchConfiguration("frontier_score_w_risk"), value_type=float),
                "max_candidate_path_len_m": ParameterValue(
                    LaunchConfiguration("frontier_max_candidate_path_len_m"), value_type=float
                ),
                "risk_check_radius_cells": ParameterValue(
                    LaunchConfiguration("frontier_risk_check_radius_cells"), value_type=int
                ),
                "recovery_failures_threshold": ParameterValue(
                    LaunchConfiguration("frontier_recovery_failures_threshold"), value_type=int
                ),
                "recovery_wait_sec": ParameterValue(LaunchConfiguration("frontier_recovery_wait_sec"), value_type=float),
            }
        ],
    )

    mission_orchestrator = Node(
        package="robot_navigation",
        executable="mission_orchestrator",
        output="screen",
        parameters=[
            {
                "autostart": ParameterValue(LaunchConfiguration("autostart_mission"), value_type=bool),
                "mapping_mode": "frontier",
                "nav2_namespace": LaunchConfiguration("nav2_namespace"),
                "manual_override_topic": LaunchConfiguration("manual_override_topic"),
                "odom_topic": "/odom",
                "boot_capture_delay_sec": ParameterValue(
                    LaunchConfiguration("boot_capture_delay_sec"), value_type=float
                ),
                "explore_timeout_sec": ParameterValue(
                    LaunchConfiguration("max_explore_time_sec"), value_type=float
                ),
                "home_retry_limit": ParameterValue(
                    LaunchConfiguration("home_retry_limit"), value_type=int
                ),
                "frontier_done_topic": LaunchConfiguration("frontier_done_topic"),
                "frontier_start_service": LaunchConfiguration("frontier_start_service"),
                "frontier_stop_service": LaunchConfiguration("frontier_stop_service"),
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

    semantic_overlay = Node(
        package="robot_navigation",
        executable="semantic_overlay",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_semantic_overlay")),
        parameters=[
            {
                "frame_id": LaunchConfiguration("global_frame"),
                "shelves_file": LaunchConfiguration("shelves_file"),
                "map_topic": LaunchConfiguration("semantic_map_topic"),
                "marker_topic": "/semantic_overlay/markers",
                "auto_align_service": "/semantic_overlay/auto_align",
                "reference_map_yaml": LaunchConfiguration("semantic_reference_map_yaml"),
                "auto_align_on_start": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_on_start"), value_type=bool
                ),
                "auto_align_retry_sec": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_retry_sec"), value_type=float
                ),
                "auto_align_min_score": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_min_score"), value_type=float
                ),
                "auto_align_yaw_search_deg": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_yaw_search_deg"), value_type=float
                ),
                "auto_align_yaw_step_deg": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_yaw_step_deg"), value_type=float
                ),
                "auto_align_translation_search_m": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_translation_search_m"), value_type=float
                ),
                "auto_align_translation_step_m": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_translation_step_m"), value_type=float
                ),
                "auto_align_sample_stride_cells": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_sample_stride_cells"), value_type=int
                ),
                "auto_align_max_reference_points": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_max_reference_points"), value_type=int
                ),
                "auto_align_occupied_threshold": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_occupied_threshold"), value_type=int
                ),
                "auto_align_global_fallback": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_global_fallback"), value_type=bool
                ),
                "auto_align_global_yaw_step_deg": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_global_yaw_step_deg"), value_type=float
                ),
                "auto_align_global_translation_step_m": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_global_translation_step_m"), value_type=float
                ),
                "auto_align_global_translation_limit_m": ParameterValue(
                    LaunchConfiguration("semantic_auto_align_global_translation_limit_m"), value_type=float
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
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("navigate_to_pose_action", default_value="/navigate_to_pose"),
            DeclareLaunchArgument("frontier_state_topic", default_value="/frontier_explorer/state"),
            DeclareLaunchArgument("frontier_done_topic", default_value="/frontier_explorer/done"),
            DeclareLaunchArgument("frontier_goal_topic", default_value="/frontier_explorer/current_goal"),
            DeclareLaunchArgument("frontier_start_service", default_value="/frontier_explorer/start"),
            DeclareLaunchArgument("frontier_stop_service", default_value="/frontier_explorer/stop"),
            DeclareLaunchArgument("autostart_frontier", default_value="false"),
            DeclareLaunchArgument("frontier_loop_rate_hz", default_value="1.5"),
            DeclareLaunchArgument("min_frontier_cluster_size", default_value="8"),
            DeclareLaunchArgument("frontier_goal_timeout_sec", default_value="35.0"),
            DeclareLaunchArgument("frontier_blacklist_ttl_sec", default_value="45.0"),
            DeclareLaunchArgument("frontier_blacklist_radius_m", default_value="0.35"),
            DeclareLaunchArgument("frontier_no_frontier_rounds_limit", default_value="5"),
            DeclareLaunchArgument("max_explore_time_sec", default_value=str(20.0 * 60.0)),
            DeclareLaunchArgument("frontier_new_area_ratio_threshold", default_value="0.01"),
            DeclareLaunchArgument("frontier_new_area_window_sec", default_value="60.0"),
            DeclareLaunchArgument("frontier_score_w_gain", default_value="1.0"),
            DeclareLaunchArgument("frontier_score_w_path", default_value="0.8"),
            DeclareLaunchArgument("frontier_score_w_risk", default_value="0.6"),
            DeclareLaunchArgument("frontier_max_candidate_path_len_m", default_value="0.0"),
            DeclareLaunchArgument("frontier_risk_check_radius_cells", default_value="2"),
            DeclareLaunchArgument("frontier_recovery_failures_threshold", default_value="3"),
            DeclareLaunchArgument("frontier_recovery_wait_sec", default_value="3.0"),
            DeclareLaunchArgument("autostart_mission", default_value="true"),
            DeclareLaunchArgument("boot_capture_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("home_retry_limit", default_value="2"),
            DeclareLaunchArgument("map_output_dir", default_value=default_maps_dir),
            DeclareLaunchArgument("map_name", default_value=""),
            DeclareLaunchArgument("map_name_prefix", default_value="run"),
            DeclareLaunchArgument("export_map", default_value="true"),
            DeclareLaunchArgument("export_resolution", default_value="0.03"),
            DeclareLaunchArgument("with_semantic_overlay", default_value="false"),
            DeclareLaunchArgument("shelves_file", default_value=default_shelves_file),
            DeclareLaunchArgument("semantic_map_topic", default_value="/map"),
            DeclareLaunchArgument("semantic_reference_map_yaml", default_value=""),
            DeclareLaunchArgument("semantic_auto_align_on_start", default_value="false"),
            DeclareLaunchArgument("semantic_auto_align_retry_sec", default_value="2.0"),
            DeclareLaunchArgument("semantic_auto_align_min_score", default_value="0.45"),
            DeclareLaunchArgument("semantic_auto_align_yaw_search_deg", default_value="12.0"),
            DeclareLaunchArgument("semantic_auto_align_yaw_step_deg", default_value="2.0"),
            DeclareLaunchArgument("semantic_auto_align_translation_search_m", default_value="1.5"),
            DeclareLaunchArgument("semantic_auto_align_translation_step_m", default_value="0.2"),
            DeclareLaunchArgument("semantic_auto_align_sample_stride_cells", default_value="4"),
            DeclareLaunchArgument("semantic_auto_align_max_reference_points", default_value="3000"),
            DeclareLaunchArgument("semantic_auto_align_occupied_threshold", default_value="60"),
            DeclareLaunchArgument("semantic_auto_align_global_fallback", default_value="true"),
            DeclareLaunchArgument("semantic_auto_align_global_yaw_step_deg", default_value="15.0"),
            DeclareLaunchArgument("semantic_auto_align_global_translation_step_m", default_value="1.0"),
            DeclareLaunchArgument("semantic_auto_align_global_translation_limit_m", default_value="6.0"),
            slam_mapping_stack,
            TimerAction(period=2.0, actions=[nav2_bringup]),
            frontier_explorer,
            mission_orchestrator,
            semantic_overlay,
            nav2_rviz,
        ]
    )
