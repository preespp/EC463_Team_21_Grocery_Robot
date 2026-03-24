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
                "-0.0449822",
                "--y",
                "0.0292244",
                "--z",
                "0.0681345",
                "--qx",
                "-0.501952",
                "--qy",
                "0.506792",
                "--qz",
                "-0.488658",
                "--qw",
                "0.502414",
                # "--roll",
                # "0.476552",
                # "--pitch",
                # "1.59059",
                # "--yaw",
                # "-2.02875",
            ],
        ),
    ]
    return LaunchDescription(nodes)
