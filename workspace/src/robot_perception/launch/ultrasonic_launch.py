from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = {
        "bus": 7,
        "rate": 100.0,
        "threshold_m": 0.20,
    }

    return LaunchDescription([

        Node(
            package="robot_perception",
            executable="distance_sensor",
            name="ultrasonic_front",
            parameters=[
               common,
               {"addr": 0x09, "name": "front"},
            ],
            output="screen",
        ),

        Node(
            package="robot_perception",
            executable="distance_sensor",
            name="ultrasonic_right",
            parameters=[
                common,
                {"addr": 0x11, "name": "right"},
            ],
            output="screen",
        ),

        Node(
           package="robot_perception",
           executable="distance_sensor",
           name="ultrasonic_left",
            parameters=[
               common,
               {"addr": 0x10, "name": "left"},
            ],
            output="screen",
        ),

        Node(
           package="robot_perception",
           executable="distance_sensor",
           name="ultrasonic_back",
            parameters=[
               common,
               {"addr": 0x12, "name": "back"},
           ],
           output="screen",
        ),
    ])
