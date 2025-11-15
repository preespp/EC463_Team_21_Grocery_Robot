## for full systems
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionServer
# from mvp_robot_msgs.action import FollowJointTrajectory
# from sensor_msgs.msg import JointState

# import numpy as np
# import time
# import spidev
# import math

# JOINT_NAMES = ["base", "shoulder", "elbow", "wrist", "gripper"]
# LIMIT_LOW =   [0,   0,   0,   0,  0]
# LIMIT_HIGH = [270, 270, 270, 270, 270]

# class ArmMotor(Node):
#     def __init__(self):
#         super().__init__("arm_motor")

#         # SPI init
#         self.spi = spidev.SpiDev()
#         self.spi.open(0, 0)   # bus 0, CE0
#         self.spi.max_speed_hz = 1000000

#         # pub joint states
#         self.pub = self.create_publisher(JointState, "joint_states", 10)
#         self.current_deg = [0]*5

#         # action server
#         self.server = ActionServer(
#             self,
#             FollowJointTrajectory,
#             "arm_controller",
#             execute_callback=self.execute_cb)

#         self.timer = self.create_timer(0.05, self.pub_joint_states)

#     def pub_joint_states(self):
#         msg = JointState()
#         msg.name = JOINT_NAMES
#         msg.position = [math.radians(x) for x in self.current_deg]
#         msg.header.stamp = self.get_clock().now().to_msg()
#         self.pub.publish(msg)

#     def spi_send(self, text):
#         buf = bytearray(text.ljust(64)[:64], 'ascii')
#         self.spi.xfer2(buf)

#     def enforce_limits(self, arr):
#         return [max(LIMIT_LOW[i], min(LIMIT_HIGH[i], arr[i])) for i in range(5)]

#     def execute_cb(self, goal):
#         traj = goal.goal.trajectory
#         for point in traj.points:

#             deg = [math.degrees(p) for p in point.positions]
#             deg = self.enforce_limits(deg)

#             cmd = f"{deg[0]} {deg[1]} {deg[2]} {deg[3]} {deg[4]}"
#             self.spi_send(cmd)

#             self.current_deg = deg
#             time.sleep(0.02)

#         return FollowJointTrajectory.Result()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmMotor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# Test SPI with ROS2
import rclpy
from rclpy.node import Node
import spidev

class ServoTest(Node):
    def __init__(self):
        super().__init__('servo_test')
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # bus=0, cs=0
        self.spi.max_speed_hz = 1000000

        self.angle = 0
        self.timer = self.create_timer(1.0, self.send_angle)

    def send_angle(self):
        self.spi.xfer([self.angle])
        self.get_logger().info(f"Sent angle: {self.angle}")
        self.angle = (self.angle + 20) % 180

def main():
    rclpy.init()
    node = ServoTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
