""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: vx300s/ee_gripper_link -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "vx300s/ee_gripper_link",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0463943",
                "--y",
                "0.0290953", # might change it to -0.011 if doesn't work originally it is 0.0290953
                "--z",
                "0.0549267",
                "--qx",
                "-0.503454",
                "--qy",
                "0.500397",
                "--qz",
                "-0.489836",
                "--qw",
                "0.50616",
                # "--roll",
                # "1.19232",
                # "--pitch",
                # "1.59171",
                # "--yaw",
                # "-2.74985",
            ],
        ),
    ]
    return LaunchDescription(nodes)
