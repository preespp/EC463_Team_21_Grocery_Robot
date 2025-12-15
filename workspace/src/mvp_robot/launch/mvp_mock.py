from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Central Node
        Node(
            package='mvp_robot',
            executable='central',
            name='central_node',
            output='screen'
        ),

        # Collision Detection Node
        Node(
            package='mvp_robot',
            executable='distance_sensor',
            name='distance_sensor_node',
            output='screen'
        ),
        
        # Camera Vision Node
        Node(
            package='mvp_robot',
            executable='camera_vision',
            name='camera_vision_node',
            output='screen'
        ),
    ])
