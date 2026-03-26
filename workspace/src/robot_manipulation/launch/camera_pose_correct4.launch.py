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
                "-0.0462321",
                "--y",
                "0.0292076",
                "--z",
                "0.059098",
                "--qx",
                "-0.503919",
                "--qy",
                "0.501676",
                "--qz",
                "-0.48578",
                "--qw",
                "0.508335",
                # "--roll",
                # "1.14739",
                # "--pitch",
                # "1.59812",
                # "--yaw",
                # "-2.69787",
            ],
        ),
    ]
    return LaunchDescription(nodes)
