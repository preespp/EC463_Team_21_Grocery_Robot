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
                "-3.27331",
                "--y",
                "0.108822",
                "--z",
                "0.433789",
                "--qx",
                "0.644203",
                "--qy",
                "-0.351641",
                "--qz",
                "0.548344",
                "--qw",
                "-0.400836",
                # "--roll",
                # "1.03704",
                # "--pitch",
                # "1.72332",
                # "--yaw",
                # "-3.05285",
            ],
        ),
    ]
    return LaunchDescription(nodes)
