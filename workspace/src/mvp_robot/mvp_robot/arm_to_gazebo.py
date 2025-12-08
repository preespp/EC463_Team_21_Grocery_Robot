#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_NAMES = [
    "joint1_base_yaw",
    "joint2_shoulder",
    "joint3_elbow",
    "joint4_wrist_roll",
    "joint5_gripper",
]


class ArmJointStateToCmd(Node):
    """
    Take desired JointState on /arm/joint_state and fan out to
    per-joint Float64 command topics:

      /arm/joint1_cmd ... /arm/joint5_cmd

    These topics are bridged into Gazebo and fed to the
    JointPositionController plugins.
    """

    def __init__(self):
        super().__init__("arm_js_to_cmd")

        self.sub = self.create_subscription(
            JointState,
            "/arm/joint_state",
            self.js_callback,
            10,
        )

        # Create per-joint publishers
        self.pub = {
            "joint1_base_yaw": self.create_publisher(Float64, "/arm/joint1_cmd", 10),
            "joint2_shoulder": self.create_publisher(Float64, "/arm/joint2_cmd", 10),
            "joint3_elbow": self.create_publisher(Float64, "/arm/joint3_cmd", 10),
            "joint4_wrist_roll": self.create_publisher(Float64, "/arm/joint4_cmd", 10),
            "joint5_gripper": self.create_publisher(Float64, "/arm/joint5_cmd", 10),
        }

        self.get_logger().info("arm_to_gazebo node started (listening on /arm/joint_state)")

    def js_callback(self, msg: JointState):
        # Build dict from name -> position for safety
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        for joint_name in JOINT_NAMES:
            if joint_name not in name_to_pos:
                continue
            pos = name_to_pos[joint_name]
            out = Float64()
            out.data = float(pos)
            self.pub[joint_name].publish(out)

        # Optional debug:
        # self.get_logger().info(str(name_to_pos))


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointStateToCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

