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

# Test I2C with 5 servo motors
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smbus2 import SMBus
import struct
import time


class ServoI2C(Node):
    def __init__(self):
        super().__init__("arm_motor")

        # Parameters can override via ROS2 launch
        self.declare_parameter("bus", 7)
        self.declare_parameter("addr", 0x08)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("joint_count", 5)

        # Load parameters
        self.bus_id = self.get_parameter("bus").value
        self.addr = self.get_parameter("addr").value
        self.rate = self.get_parameter("rate").value
        self.joint_count = self.get_parameter("joint_count").value

        # Open I2C bus
        try:
            self.bus = SMBus(self.bus_id)
        except FileNotFoundError:
            self.get_logger().error(f"Could not open /dev/i2c-{self.bus_id}")
            raise

        self.get_logger().info(
            f"Started I2C Servo Sender on /dev/i2c-{self.bus_id}, addr=0x{self.addr:02X}"
        )

        # Latest joint angles (default zeros)
        self.joint_angles = [0.0] * self.joint_count

        # Subscribe to JointState topic
        self.subscription = self.create_subscription(
            JointState,
            "/arm/joint_states",
            self.joint_callback,
            10
        )

        # Timer to send I2C packets periodically
        self.timer = self.create_timer(1.0 / self.rate, self.send_i2c_packet)

    def joint_callback(self, msg: JointState):
        # Use the first N joint positions
        for i in range(min(self.joint_count, len(msg.position))):
            self.joint_angles[i] = float(msg.position[i])

    def send_i2c_packet(self):
        # Pack the joint angles as <ffffff....
        packet = struct.pack("<" + "f" * self.joint_count, *self.joint_angles)

        try:
            self.bus.write_i2c_block_data(self.addr, 0x00, list(packet))
        except Exception as e:
            self.get_logger().warn(f"I2C send failed: {e}")
            return

        self.get_logger().info(
            "Sent angles: " + ", ".join(f"{a:.2f}" for a in self.joint_angles)
        )

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoI2C()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
