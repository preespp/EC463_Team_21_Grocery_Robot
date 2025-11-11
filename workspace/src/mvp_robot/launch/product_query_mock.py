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

        # Product Database Query Node
        Node(
            package='mvp_robot',
            executable='product_db_query',
            name='product_db_node',
            output='screen'
        ),

        # UI Input Node
        Node(
            package='mvp_robot',
            executable='ui_input',
            name='ui_input_node',
            output='screen'
        )
    ])
