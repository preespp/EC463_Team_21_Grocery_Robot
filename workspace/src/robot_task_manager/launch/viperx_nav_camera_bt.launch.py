from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for parent in [current.parent, *current.parents]:
        if (parent / "Maps").exists() and (parent / "workspace").exists():
            return parent
    return Path.cwd()


def generate_launch_description():
    repo_root = _find_repo_root()
    navigation_share = Path(get_package_share_directory("robot_navigation"))

    default_pbstream = str(repo_root / "Maps" / "testmapMain.pbstream")
    default_map_yaml = str(repo_root / "Maps" / "testmapMain.yaml")
    default_nav2_params = str(navigation_share / "config" / "nav2_params_smac_mppi_omni.yaml")
    default_ekf_params = str(navigation_share / "config" / "ekf_odom_base_imu.yaml")
    default_semantic_map = str(navigation_share / "config" / "semantic_map_testmapMain.yaml")

    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_manipulation"), "launch", "vx300_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_name": robot_name,
            "motor_port": LaunchConfiguration("motor_port"),
            "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
            "use_viperx_arm_server": "true",
            "use_viperx_preview": LaunchConfiguration("use_viperx_preview"),
            "use_auto_pick": "false",
            "xs_driver_logging_level": LaunchConfiguration("xs_driver_logging_level"),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_navigation"),
                    "launch",
                    "nav2_localization_stack.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_navigation")),
        launch_arguments={
            "hostname": LaunchConfiguration("hostname"),
            "udp_receiver_ip": LaunchConfiguration("udp_receiver_ip"),
            "serial_port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "cmd_topics": LaunchConfiguration("cmd_topics"),
            "telemetry_enabled": LaunchConfiguration("telemetry_enabled"),
            "bridge_max_linear_speed": LaunchConfiguration("bridge_max_linear_speed"),
            "bridge_max_lateral_speed": LaunchConfiguration("bridge_max_lateral_speed"),
            "bridge_max_yaw_speed": LaunchConfiguration("bridge_max_yaw_speed"),
            "bridge_axis_deadband": LaunchConfiguration("bridge_axis_deadband"),
            "left_switch": LaunchConfiguration("left_switch"),
            "right_switch": LaunchConfiguration("right_switch"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "fallback_odom": LaunchConfiguration("fallback_odom"),
            "pbstream_file": LaunchConfiguration("pbstream_file"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "with_semantic_map": LaunchConfiguration("with_semantic_map"),
            "semantic_map_file": LaunchConfiguration("semantic_map_file"),
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "nav2_namespace": LaunchConfiguration("nav2_namespace"),
            "use_ekf": LaunchConfiguration("use_ekf"),
            "ekf_params_file": LaunchConfiguration("ekf_params_file"),
            "cartographer_config_basename": LaunchConfiguration("cartographer_config_basename"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "sick_tf_publish_rate": LaunchConfiguration("sick_tf_publish_rate"),
            "imu_udp_port": LaunchConfiguration("imu_udp_port"),
            "scandataformat": LaunchConfiguration("scandataformat"),
            "send_sopas_start_stop_cmd": LaunchConfiguration("send_sopas_start_stop_cmd"),
            "host_set_frecho_filter": LaunchConfiguration("host_set_frecho_filter"),
            "host_set_lfp_angle_range_filter": LaunchConfiguration(
                "host_set_lfp_angle_range_filter"
            ),
            "host_set_lfp_interval_filter": LaunchConfiguration(
                "host_set_lfp_interval_filter"
            ),
            "laserscan_layer_filter": LaunchConfiguration("laserscan_layer_filter"),
            "lidar_x": LaunchConfiguration("lidar_x"),
            "lidar_y": LaunchConfiguration("lidar_y"),
            "lidar_z": LaunchConfiguration("lidar_z"),
            "lidar_roll": LaunchConfiguration("lidar_roll"),
            "lidar_pitch": LaunchConfiguration("lidar_pitch"),
            "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            "imu_x": LaunchConfiguration("imu_x"),
            "imu_y": LaunchConfiguration("imu_y"),
            "imu_z": LaunchConfiguration("imu_z"),
            "imu_roll": LaunchConfiguration("imu_roll"),
            "imu_pitch": LaunchConfiguration("imu_pitch"),
            "imu_yaw": LaunchConfiguration("imu_yaw"),
            "with_base_link_crop": LaunchConfiguration("with_base_link_crop"),
            "crop_box_frame": LaunchConfiguration("crop_box_frame"),
            "crop_min_x": LaunchConfiguration("crop_min_x"),
            "crop_max_x": LaunchConfiguration("crop_max_x"),
            "crop_min_y": LaunchConfiguration("crop_min_y"),
            "crop_max_y": LaunchConfiguration("crop_max_y"),
            "crop_min_z": LaunchConfiguration("crop_min_z"),
            "crop_max_z": LaunchConfiguration("crop_max_z"),
            "with_nav2_rviz": LaunchConfiguration("with_nav2_rviz"),
        }.items(),
    )

    camera_node = Node(
        package="robot_vision",
        executable="camera_vision",
        name="camera_vision",
        output="screen",
        parameters=[
            {
                "show_live_window": LaunchConfiguration("show_live_window"),
                "parent_frame": LaunchConfiguration("camera_parent_frame"),
                "camera_mount_frame": LaunchConfiguration("camera_mount_frame"),
                "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
            }
        ],
    )

    bt_node = Node(
        package="robot_task_manager",
        executable="bt_executor_viperX",
        name="bt_executor_viperX",
        output="screen",
        parameters=[
            {
                "skip_navigation": LaunchConfiguration("skip_navigation"),
                "detection_min_confidence": LaunchConfiguration("detection_min_confidence"),
            }
        ],
    )

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_perception"), "launch", "ultrasonic_launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("include_ultrasonic")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value="vx300s"),
            DeclareLaunchArgument("robot_name", default_value=robot_model),
            DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
            DeclareLaunchArgument("use_viperx_preview", default_value="false"),
            DeclareLaunchArgument("xs_driver_logging_level", default_value="INFO"),
            DeclareLaunchArgument("launch_navigation", default_value="true"),
            DeclareLaunchArgument("skip_navigation", default_value="false"),
            DeclareLaunchArgument("include_ultrasonic", default_value="false"),
            DeclareLaunchArgument("show_live_window", default_value="false"),
            DeclareLaunchArgument(
                "camera_parent_frame",
                default_value=[robot_name, "/ee_gripper_link"],
            ),
            DeclareLaunchArgument("camera_mount_frame", default_value="camera_mount_frame"),
            DeclareLaunchArgument("camera_optical_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("detection_min_confidence", default_value="0.40"),
            DeclareLaunchArgument("hostname", default_value="192.168.8.150"),
            DeclareLaunchArgument("udp_receiver_ip", default_value="192.168.8.249"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("cmd_topics", default_value='["/cmd_vel"]'),
            DeclareLaunchArgument("telemetry_enabled", default_value="true"),
            DeclareLaunchArgument("bridge_max_linear_speed", default_value="3.0"),
            DeclareLaunchArgument("bridge_max_lateral_speed", default_value="3.0"),
            DeclareLaunchArgument("bridge_max_yaw_speed", default_value="12.566370614359172"),
            DeclareLaunchArgument("bridge_axis_deadband", default_value="0.05"),
            DeclareLaunchArgument("left_switch", default_value="1"),
            DeclareLaunchArgument("right_switch", default_value="1"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_raw"),
            DeclareLaunchArgument("fallback_odom", default_value="false"),
            DeclareLaunchArgument("pbstream_file", default_value=default_pbstream),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("with_semantic_map", default_value="true"),
            DeclareLaunchArgument("semantic_map_file", default_value=default_semantic_map),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("nav2_namespace", default_value=""),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            DeclareLaunchArgument(
                "cartographer_config_basename",
                default_value="pico_2d_localization.lua",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/sick_scansegment_xd/imu"),
            DeclareLaunchArgument("sick_tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("imu_udp_port", default_value="7503"),
            DeclareLaunchArgument("scandataformat", default_value="2"),
            DeclareLaunchArgument("send_sopas_start_stop_cmd", default_value="0"),
            DeclareLaunchArgument("host_set_frecho_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_angle_range_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_interval_filter", default_value="0"),
            DeclareLaunchArgument("laserscan_layer_filter", default_value="0"),
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
            DeclareLaunchArgument("with_base_link_crop", default_value="false"),
            DeclareLaunchArgument("crop_box_frame", default_value="base_link"),
            DeclareLaunchArgument("crop_min_x", default_value="-0.2540"),
            DeclareLaunchArgument("crop_max_x", default_value="0.1397"),
            DeclareLaunchArgument("crop_min_y", default_value="-0.2794"),
            DeclareLaunchArgument("crop_max_y", default_value="0.2794"),
            DeclareLaunchArgument("crop_min_z", default_value="-1.0"),
            DeclareLaunchArgument("crop_max_z", default_value="1.0"),
            DeclareLaunchArgument("with_nav2_rviz", default_value="false"),
            moveit_launch,
            navigation_launch,
            camera_node,
            bt_node,
            ultrasonic_launch,
        ]
    )
