# arm_control/arm_joint_demo.py
#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ArmJointDemo(Node):
    def __init__(self):
        super().__init__("arm_joint_demo")
        self.pub = self.create_publisher(Float32MultiArray, "/arm/joint_cmd", 10)
        self.t = 0.0
        self.dt = 0.05  # 20 Hz
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("ArmJointDemo started, publishing to /arm/joint_cmd")

    def update(self):
        self.t += self.dt
        msg = Float32MultiArray()

        # Simple periodic trajectories in radians
        base = 0.5 * math.sin(0.3 * self.t)         # joint1_base_yaw
        shoulder = 0.5 * math.sin(0.2 * self.t)     # joint2_shoulder
        elbow = 1.0 + 0.3 * math.sin(0.25 * self.t) # joint3_elbow
        wrist = 0.8 * math.sin(0.4 * self.t)        # joint4_wrist_roll
        grip = 0.4 * (0.5 + 0.5 * math.sin(0.5 * self.t))  # 0..0.4

        msg.data = [base, shoulder, elbow, wrist, grip]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

