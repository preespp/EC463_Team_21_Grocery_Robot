from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


POINTCLOUD_CONFIG = (
    "coordinateNotation=3 updateMethod=0 "
    "fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector "
    "echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 "
    "reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 "
    "topic=/cloud_all_fields_fullframe frameid=lidar_link publish=1"
)


def generate_launch_description():
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "config", "ekf_odom_base_imu.yaml"]
    )

    # sick_picoscan.launch.py forwards sys.argv directly to sick_generic_caller.
    # Launching the caller node directly guarantees these args are applied.
    sick_driver = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        output="screen",
        arguments=[
            PathJoinSubstitution(
                [FindPackageShare("sick_scan_xd"), "launch", "sick_picoscan.launch"]
            ),
            ["hostname:=", LaunchConfiguration("hostname")],
            ["udp_receiver_ip:=", LaunchConfiguration("udp_receiver_ip")],
            "publish_frame_id:=lidar_link",
            "publish_imu_frame_id:=lidar_link",
            ["tf_publish_rate:=", LaunchConfiguration("sick_tf_publish_rate")],
            ["imu_udp_port:=", LaunchConfiguration("imu_udp_port")],
            ["scandataformat:=", LaunchConfiguration("scandataformat")],
            ["send_sopas_start_stop_cmd:=", LaunchConfiguration("send_sopas_start_stop_cmd")],
            ["host_set_FREchoFilter:=", LaunchConfiguration("host_set_frecho_filter")],
            [
                "host_set_LFPangleRangeFilter:=",
                LaunchConfiguration("host_set_lfp_angle_range_filter"),
            ],
            [
                "host_set_LFPintervalFilter:=",
                LaunchConfiguration("host_set_lfp_interval_filter"),
            ],
            ["laserscan_layer_filter:=", LaunchConfiguration("laserscan_layer_filter")],
            "custom_pointclouds:=cloud_all_fields_fullframe",
            ["cloud_all_fields_fullframe:=", POINTCLOUD_CONFIG],
            "publish_laserscan_fullframe_topic:=/scan_fullframe",
            ["imu_topic:=", LaunchConfiguration("imu_topic")],
        ],
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_static_tf",
        output="screen",
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
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_navigation"), "launch", "cartographer_mapping.launch.py"]
            )
        ),
        launch_arguments={
            "configuration_basename": LaunchConfiguration("cartographer_config_basename"),
        }.items(),
    )

    serial_bridge = Node(
        package="robot_navigation",
        executable="nav2_serial_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                "cmd_topics": LaunchConfiguration("cmd_topics"),
                "telemetry_enabled": ParameterValue(
                    LaunchConfiguration("telemetry_enabled"), value_type=bool
                ),
                "left_switch": ParameterValue(LaunchConfiguration("left_switch"), value_type=int),
                "right_switch": ParameterValue(LaunchConfiguration("right_switch"), value_type=int),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "publish_tf": False,
                "fallback_odom": ParameterValue(
                    LaunchConfiguration("fallback_odom"), value_type=bool
                ),
            }
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_ekf")),
        parameters=[LaunchConfiguration("ekf_params_file")],
        remappings=[("odometry/filtered", "/odom")],
    )

    collision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_perception"), "launch", "ultrasonic_launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("with_collision")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("with_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hostname", default_value="192.168.8.150"),
            DeclareLaunchArgument("udp_receiver_ip", default_value="192.168.8.249"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument(
                "cmd_topics",
                default_value='["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]',
            ),
            DeclareLaunchArgument("telemetry_enabled", default_value="true"),
            DeclareLaunchArgument("left_switch", default_value="1"),
            DeclareLaunchArgument("right_switch", default_value="1"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_raw"),
            DeclareLaunchArgument("fallback_odom", default_value="false"),
            DeclareLaunchArgument("use_ekf", default_value="true"),
            DeclareLaunchArgument("ekf_params_file", default_value=ekf_params),
            DeclareLaunchArgument(
                "cartographer_config_basename",
                default_value="pico_2d_mapping_quality.lua",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/sick_scansegment_xd/imu"),
            DeclareLaunchArgument("sick_tf_publish_rate", default_value="0.0"),
            DeclareLaunchArgument("imu_udp_port", default_value="7503"),
            DeclareLaunchArgument("scandataformat", default_value="2"),
            # sick_generic_caller parses bool launch overrides as numeric strings.
            DeclareLaunchArgument("send_sopas_start_stop_cmd", default_value="0"),
            DeclareLaunchArgument("host_set_frecho_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_angle_range_filter", default_value="0"),
            DeclareLaunchArgument("host_set_lfp_interval_filter", default_value="0"),
            DeclareLaunchArgument("laserscan_layer_filter", default_value="0"),
            DeclareLaunchArgument("lidar_x", default_value="0.254"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("with_collision", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="false"),
            sick_driver,
            lidar_static_tf,
            cartographer_launch,
            serial_bridge,
            ekf_node,
            collision_launch,
            rviz,
        ]
    )
