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
                "-0.0475461",
                "--y",
                "0.0299081",
                "--z",
                "0.0594435",
                "--qx",
                "-0.501915",
                "--qy",
                "0.504069",
                "--qz",
                "-0.485667",
                "--qw",
                "0.508058",
                # "--roll",
                # "1.03844",
                # "--pitch",
                # "1.59446",
                # "--yaw",
                # "-2.58469",
            ],
        ),
    ]
    return LaunchDescription(nodes)
