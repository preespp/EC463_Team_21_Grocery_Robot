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
                "-0.98129",
                "--y",
                "-0.256618",
                "--z",
                "-0.0501378",
                "--qx",
                "-0.537232",
                "--qy",
                "0.462096",
                "--qz",
                "-0.464387",
                "--qw",
                "0.53122",
                # "--roll",
                # "1.54043",
                # "--pitch",
                # "1.71293",
                # "--yaw",
                # "-3.11955",
            ],
        ),
    ]
    return LaunchDescription(nodes)
