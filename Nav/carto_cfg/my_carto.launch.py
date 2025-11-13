# ~/carto_cfg/my_carto.launch.py  
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/home/grocerybot/carto_cfg',
                '-configuration_basename', 'pico_2d.lua'
            ],
            #remappings=[('scan', '/scan_fullframe')
            remappings=[('points2', '/cloud_all_fields_fullframe'),
            ('imu',     '/sick_scansegment_xd/imu'),
            ]

        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='carto_grid',
            output='screen',
            parameters=[{'resolution': 0.05, 'publish_period_sec': 1.0}]
        ),
    ])

