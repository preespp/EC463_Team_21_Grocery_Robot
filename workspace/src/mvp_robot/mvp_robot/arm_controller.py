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

# ik version
"""
#!/usr/bin/env python3
import math
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

JOINT_LIMITS = {
    "joint1_base_yaw":   (-2.3562,  2.3562),
    "joint2_shoulder":   (-1.5708,  1.5708),
    "joint3_elbow":      (0.0,      2.3562),
    "joint4_wrist_roll": (-2.3562,  2.3562),
    "joint5_gripper":    (0.0,      0.8),
}

# Link lengths for simple IK model (meters) – approximate your URDF
H_SHOULDER = 0.25  # height of shoulder joint above base origin
L1 = 0.30          # upper arm length
L2 = 0.30          # forearm length

POS_STEP = 0.02    # meters per key press


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def get_key(settings, timeout=0.05):
    """"""Non-blocking read of a single key.""""""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmKeyboardIK(Node):
    """"""
    End-effector position keyboard teleop with simple IK.

    Keys move the desired end-effector position (x, y, z) and we solve IK
    for the 5-DOF arm, then publish sensor_msgs/JointState on /arm/joint_state.
    """"""

    def __init__(self):
        super().__init__("arm_keyboard_ik")
        self.publisher = self.create_publisher(JointState, "/arm/joint_state", 10)

        # Start with some reachable default target (x, y, z)
        self.target_pos = [0.4, 0.0, 0.4]

        # Initial guess for joints (will be overwritten by IK)
        self.positions = [0.0] * len(JOINT_NAMES)
        self.update_ik()  # initialize joints

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.get_logger().info(
            "ArmKeyboardIK started. Controls (move end-effector):\n"
            "  w / s : +x / -x\n"
            "  a / d : +y / -y\n"
            "  r / f : +z / -z\n"
            "  q     : quit\n"
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(self.positions)
        self.publisher.publish(msg)

    def process_key(self, key: str) -> bool:
        """"""
        Process one keystroke.
        Returns False if we should exit.
        """"""
        if key in ("q", "Q"):
            self.get_logger().info("Quitting ArmKeyboardIK.")
            return False

        moved = False

        if key == "w":
            self.target_pos[0] += POS_STEP
            moved = True
        elif key == "s":
            self.target_pos[0] -= POS_STEP
            moved = True
        elif key == "a":
            self.target_pos[1] += POS_STEP
            moved = True
        elif key == "d":
            self.target_pos[1] -= POS_STEP
            moved = True
        elif key == "r":
            self.target_pos[2] += POS_STEP
            moved = True
        elif key == "f":
            self.target_pos[2] -= POS_STEP
            moved = True

        if moved:
            if self.update_ik():
                x, y, z = self.target_pos
                self.get_logger().info(
                    f"Target pos -> x={x:.3f}, y={y:.3f}, z={z:.3f}"
                )
            else:
                self.get_logger().warn("IK failed for requested position.")

        return True

    def update_ik(self) -> bool:
        """"""
        Compute IK for current target_pos and update self.positions.

        Simple analytic IK for a 3-DOF arm:
          - joint1: yaw (base) aligns XY
          - joint2: shoulder
          - joint3: elbow
          - joint4: wrist roll (kept at 0)
          - joint5: gripper (kept at 0)

        Returns True if successful, False otherwise.
        """"""
        x, y, z = self.target_pos

        # Base yaw
        theta1 = math.atan2(y, x)

        # Position relative to shoulder joint
        dx = x
        dy = y
        dz = z - H_SHOULDER

        # Distance from shoulder to target
        d = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Workspace clamp to keep solvable
        max_reach = L1 + L2 - 1e-4
        min_reach = abs(L1 - L2) + 1e-4

        if d > max_reach or d < min_reach:
            # Clamp position back into reachable shell
            if d == 0.0:
                return False
            scale = max_reach / d if d > max_reach else min_reach / d
            dx *= scale
            dy *= scale
            dz *= scale
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            # Update target_pos to clamped value (for transparency)
            self.target_pos = [dx, dy, dz + H_SHOULDER]

        # Planar radius and height (in plane of the arm)
        r = math.sqrt(dx * dx + dy * dy)
        z_eff = dz

        # Law of cosines for elbow
        cos_elbow = (r * r + z_eff * z_eff - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        cos_elbow = clamp(cos_elbow, -1.0, 1.0)
        theta3 = math.acos(cos_elbow)  # elbow-down configuration

        # Shoulder angle
        k1 = L1 + L2 * math.cos(theta3)
        k2 = L2 * math.sin(theta3)
        theta2 = math.atan2(z_eff, r) - math.atan2(k2, k1)

        # Keep wrist roll and gripper neutral for now
        theta4 = 0.0
        theta5 = 0.0

        candidate = [theta1, theta2, theta3, theta4, theta5]

        # Enforce joint limits to avoid "breaking" the arm
        for i, name in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[name]
            val = candidate[i]
            clamped_val = clamp(val, lo, hi)
            if clamped_val != val:
                self.get_logger().warn(
                    f"{name} IK value {val:.2f} out of range, "
                    f"clamped to [{lo:.2f}, {hi:.2f}] -> {clamped_val:.2f}"
                )
            candidate[i] = clamped_val

        self.positions = candidate
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardIK()
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

"""

