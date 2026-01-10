from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = {
        "bus": 7,
        "rate": 20.0,
        "threshold_m": 0.30,
    }

    return LaunchDescription([

        Node(
            package="robot_perception",
            executable="ultrasonic_i2c_node",
            name="ultrasonic_front",
            parameters=[
                common,
                {"addr": 0x09, "name": "front"},
            ],
            output="screen",
        ),

        Node(
            package="robot_perception",
            executable="ultrasonic_i2c_node",
            name="ultrasonic_right",
            parameters=[
                common,
                {"addr": 0x0A, "name": "right"},
            ],
            output="screen",
        ),

        Node(
            package="robot_perception",
            executable="ultrasonic_i2c_node",
            name="ultrasonic_left",
            parameters=[
                common,
                {"addr": 0x0B, "name": "left"},
            ],
            output="screen",
        ),

        Node(
            package="robot_perception",
            executable="ultrasonic_i2c_node",
            name="ultrasonic_back",
            parameters=[
                common,
                {"addr": 0x0C, "name": "back"},
            ],
            output="screen",
        ),
    ])
