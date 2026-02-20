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


POINTCLOUD_CONFIG = (
    "coordinateNotation=3 updateMethod=0 "
    "fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector "
    "echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 "
    "reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 "
    "topic=/cloud_all_fields_fullframe frameid=base_link publish=1"
)


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

    default_pbstream = str(maps_dir / "testmap1.pbstream")
    default_map_yaml = str(maps_dir / "testmap1.yaml")
    default_nav2_params = str(package_share / "config" / "nav2_params_cartographer.yaml")

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
            "publish_frame_id:=base_link",
            "publish_imu_frame_id:=base_link",
            "custom_pointclouds:=cloud_all_fields_fullframe",
            ["cloud_all_fields_fullframe:=", POINTCLOUD_CONFIG],
            "publish_laserscan_fullframe_topic:=/scan_fullframe",
            "imu_topic:=/sick_scansegment_xd/imu",
        ],
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

    cartographer_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_navigation"),
                    "launch",
                    "cartographer_localization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "load_state_filename": LaunchConfiguration("pbstream_file"),
            "publish_occupancy_grid": "false",
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "yaml_filename": LaunchConfiguration("map_yaml"),
            }
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
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
        condition=IfCondition(LaunchConfiguration("with_nav2_rviz")),
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
            DeclareLaunchArgument("pbstream_file", default_value=default_pbstream),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("with_nav2_rviz", default_value="false"),
            sick_driver,
            serial_bridge,
            cartographer_localization,
            map_server,
            lifecycle_manager,
            TimerAction(period=2.0, actions=[nav2_bringup]),
            nav2_rviz,
        ]
    )
