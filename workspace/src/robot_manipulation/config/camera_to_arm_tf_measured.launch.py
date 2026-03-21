"""Static transform publishers for the hand-measured eye-in-hand camera mount."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=[
                    "--frame-id",
                    "vx300s/ee_gripper_link",
                    "--child-frame-id",
                    "camera_mount_frame",
                    "--x",
                    "-0.0635",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0635",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="log",
                arguments=[
                    "--frame-id",
                    "camera_mount_frame",
                    "--child-frame-id",
                    "camera_color_optical_frame",
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "-1.57079632679",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "-1.57079632679",
                ],
            ),
        ]
    )
