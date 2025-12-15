'''
This version is using keyboard to control each joint manually (Forward Kinematics)
'''
#!/usr/bin/env python3
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "joint1_base_yaw",
    "joint2_shoulder",
    "joint3_elbow",
    "joint4_wrist_roll",
    "joint5_gripper",
]

# From URDF limits (radians)
JOINT_LIMITS = {
    JOINT_NAMES[0] :   (-2.3562,  2.3562),
    JOINT_NAMES[1] :   (-1.5708,  1.5708),
    JOINT_NAMES[2] :      (0.0,      2.3562),
    JOINT_NAMES[3] : (-2.3562,  2.3562),
    JOINT_NAMES[4] :    (0.0,      0.8),
}

STEP = 0.05  # radians per key press

SERVO_KEYS = {
    "1": (0, -STEP), "!": (0, +STEP),
    "2": (1, -STEP), "@": (1, +STEP),
    "3": (2, -STEP), "#": (2, +STEP),
    "4": (3, -STEP), "$": (3, +STEP),
    "5": (4, -STEP), "%": (4, +STEP),
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def get_key(settings, timeout=0.05):
    """Non-blocking read of a single key."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmKeyboardFK(Node):
    """
    Joint-space keyboard teleop for 5-DOF arm.

    Publishes sensor_msgs/JointState on /arm/joint_state.
    """

    def __init__(self):
        super().__init__("arm_keyboard_fk")
        self.publisher = self.create_publisher(JointState, "/arm/joint_state", 10)

        # Start at all zeros (within limits)
        self.positions = [0.0] * len(JOINT_NAMES)

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.get_logger().info(
            "ArmKeyboardFK started. Controls:\n"
            "  1 / ! : joint1 - / +\n"
            "  2 / @ : joint2 - / +\n"
            "  3 / # : joint3 - / +\n"
            "  4 / $ : joint4 - / +\n"
            "  5 / % : joint5 - / +\n"
            "  q     : quit\n"
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.positions)
        self.publisher.publish(msg)

    def process_key(self, key: str) -> bool:
        """
        Process one keystroke.
        Returns False if we should exit.
        """
        if key in ("q", "Q"):
            self.get_logger().info("Quitting ArmKeyboardFK.")
            return False

        if key in SERVO_KEYS:
            joint_index, delta = SERVO_KEYS[key]
            name = JOINT_NAMES[joint_index]
            lo, hi = JOINT_LIMITS[name]
            new_val = self.positions[joint_index] + delta
            clamped = clamp(new_val, lo, hi)
            if clamped != new_val:
                self.get_logger().warn(
                    f"{name} hit limit [{lo:.2f}, {hi:.2f}], "
                    f"requested {new_val:.2f}, clamped to {clamped:.2f}"
                )
            self.positions[joint_index] = clamped
            self.get_logger().info(
                f"{name} -> {self.positions[joint_index]:.3f} rad"
            )

        return True


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardFK()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.05)
            if key:
                if not node.process_key(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

'''
Basic Closed Loop Autonomous Motion (Looping motion)
'''
# #!/usr/bin/env python3
# import math
# from enum import Enum

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

# # Robot configuration
# JOINT_NAMES = [
#     "joint1_base_yaw",
#     "joint2_shoulder",
#     "joint3_elbow",
#     "joint4_wrist_roll",
#     "joint5_gripper",
# ]

# JOINT_LIMITS = {
#     "joint1_base_yaw": (-2.3562, 2.3562),
#     "joint2_shoulder": (-1.5708, 1.5708),
#     "joint3_elbow": (0.0, 2.3562),
#     "joint4_wrist_roll": (-2.3562, 2.3562),
#     "joint5_gripper": (0.0, 0.8),
# }

# # Arm geometry (meters)
# H_SHOULDER = 0.25
# L1 = 0.30
# L2 = 0.30

# # Control
# PUBLISH_RATE = 20.0  # Hz

# # Utilities
# def clamp(val, lo, hi):
#     return max(lo, min(hi, val))

# class ArmState(Enum):
#     MOVE_HOME = 0
#     MOVE_PICK = 1
#     CLOSE_GRIPPER = 2
#     MOVE_PLACE = 3
#     OPEN_GRIPPER = 4

# class ArmAutonomousIK(Node):

#     def __init__(self):
#         super().__init__("arm_autonomous_ik")

#         self.publisher = self.create_publisher(
#             JointState, "/arm/joint_state", 10
#         )

#         # Waypoints (edit these!)
#         self.home_pos  = [0.35,  0.00, 0.45]
#         self.pick_pos  = [0.45,  0.15, 0.20]
#         self.place_pos = [0.30, -0.25, 0.30]

#         self.target_pos = list(self.home_pos)
#         self.positions = [0.0] * len(JOINT_NAMES)

#         self.state = ArmState.MOVE_HOME
#         self.state_start_time = self.get_clock().now()

#         self.update_ik()

#         self.timer = self.create_timer(
#             1.0 / PUBLISH_RATE,
#             self.control_loop
#         )

#         self.get_logger().info("Autonomous Arm IK Node Started")

#     # Main control loop
#     def control_loop(self):
#         self.update_autonomy()

#         msg = JointState()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.name = list(JOINT_NAMES)
#         msg.position = list(self.positions)
#         self.publisher.publish(msg)

#     # Autonomous state machine
#     def update_autonomy(self):
#         now = self.get_clock().now()
#         elapsed = (now - self.state_start_time).nanoseconds * 1e-9

#         if self.state == ArmState.MOVE_HOME:
#             self.target_pos = self.home_pos
#             self.update_ik()
#             if elapsed > 2.0:
#                 self.transition(ArmState.MOVE_PICK)

#         elif self.state == ArmState.MOVE_PICK:
#             self.target_pos = self.pick_pos
#             self.update_ik()
#             if elapsed > 2.0:
#                 self.transition(ArmState.CLOSE_GRIPPER)

#         elif self.state == ArmState.CLOSE_GRIPPER:
#             self.positions[4] = JOINT_LIMITS["joint5_gripper"][1]
#             if elapsed > 1.0:
#                 self.transition(ArmState.MOVE_PLACE)

#         elif self.state == ArmState.MOVE_PLACE:
#             self.target_pos = self.place_pos
#             self.update_ik()
#             if elapsed > 2.0:
#                 self.transition(ArmState.OPEN_GRIPPER)

#         elif self.state == ArmState.OPEN_GRIPPER:
#             self.positions[4] = JOINT_LIMITS["joint5_gripper"][0]
#             if elapsed > 1.0:
#                 self.transition(ArmState.MOVE_HOME)

#     def transition(self, new_state):
#         self.state = new_state
#         self.state_start_time = self.get_clock().now()
#         self.get_logger().info(f"State -> {new_state.name}")

#     def update_ik(self) -> bool:
#         x, y, z = self.target_pos

#         # Base yaw
#         theta1 = math.atan2(y, x)

#         # Relative to shoulder
#         dx = x
#         dy = y
#         dz = z - H_SHOULDER

#         d = math.sqrt(dx*dx + dy*dy + dz*dz)

#         max_reach = L1 + L2 - 1e-4
#         min_reach = abs(L1 - L2) + 1e-4

#         if d < min_reach or d > max_reach:
#             d = clamp(d, min_reach, max_reach)

#         r = math.sqrt(dx*dx + dy*dy)
#         z_eff = dz

#         cos_elbow = (r*r + z_eff*z_eff - L1*L1 - L2*L2) / (2*L1*L2)
#         cos_elbow = clamp(cos_elbow, -1.0, 1.0)

#         theta3 = math.acos(cos_elbow)

#         k1 = L1 + L2 * math.cos(theta3)
#         k2 = L2 * math.sin(theta3)

#         theta2 = math.atan2(z_eff, r) - math.atan2(k2, k1)

#         theta4 = 0.0
#         theta5 = self.positions[4]

#         candidate = [theta1, theta2, theta3, theta4, theta5]

#         for i, name in enumerate(JOINT_NAMES):
#             lo, hi = JOINT_LIMITS[name]
#             candidate[i] = clamp(candidate[i], lo, hi)

#         self.positions = candidate
#         return True

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmAutonomousIK()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
