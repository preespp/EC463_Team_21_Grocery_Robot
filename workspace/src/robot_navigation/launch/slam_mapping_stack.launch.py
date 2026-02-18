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
    "topic=/cloud_all_fields_fullframe frameid=base_link publish=1"
)


def generate_launch_description():
    sick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sick_scan_xd"), "launch", "sick_picoscan.launch.py"]
            )
        ),
        launch_arguments={
            "hostname": LaunchConfiguration("hostname"),
            "udp_receiver_ip": LaunchConfiguration("udp_receiver_ip"),
            "publish_frame_id": "base_link",
            "publish_imu_frame_id": "base_link",
            "custom_pointclouds": "cloud_all_fields_fullframe",
            "cloud_all_fields_fullframe": POINTCLOUD_CONFIG,
            "publish_laserscan_fullframe_topic": "/scan_fullframe",
            "imu_topic": "/sick_scansegment_xd/imu",
        }.items(),
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_navigation"), "launch", "cartographer_mapping.launch.py"]
            )
        )
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
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "publish_tf": False,
            }
        ],
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
            DeclareLaunchArgument("with_collision", default_value="false"),
            DeclareLaunchArgument("with_rviz", default_value="false"),
            sick_launch,
            cartographer_launch,
            serial_bridge,
            collision_launch,
            rviz,
        ]
    )
