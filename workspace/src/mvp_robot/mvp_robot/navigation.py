#!/usr/bin/env python3
"""
ROS2 node: WASD teleop that publishes mecanum velocities as geometry_msgs/Twist.

- Reads keyboard input from the terminal (WASD + Q/E + space/x/z/c).
- Publishes {vx, vy, w} as a Twist:
    linear.x = vx
    linear.y = vy
    angular.z = w

Key mapping:
  w/s : forward/backward       (vx)
  a/d : left/right strafe     (vy)
  q/e : rotate left/right     (w, in-place rotation)
  z   : forward-left          (vx > 0, vy > 0)
  c   : forward-right         (vx > 0, vy < 0)
  space or x : stop

ROS2 parameters (override with --ros-args -p ...):
  cmd_vel_topic (string): topic name (default: "mecanum_cmd_vel")
  rate          (float) : keyboard polling / publish rate in Hz (default: 50.0)
  lin_speed     (float) : linear speed (m/s) for WASD (default: 0.52)
  rot_speed     (float) : angular speed (rad/s) for Q/E (default: 0.683)
"""

import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key_nonblocking():
    """Return one character from stdin if available, else None (non-blocking)."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if sys.stdin in rlist:
        ch = sys.stdin.read(1)
        return ch
    return None


def key_to_cmd(key, lin_speed, rot_speed):
    """
    Map keys to (vx, vy, w).

    Returns:
      (vx, vy, w) if this key changes motion,
      None if the key should be ignored (no change).
    """
    k = key.lower()

    # Pure translations
    if k == "w":
        return lin_speed, 0.0, 0.0
    if k == "s":
        return -lin_speed, 0.0, 0.0
    if k == "a":
        return 0.0, lin_speed, 0.0          # left strafe (vy > 0)
    if k == "d":
        return 0.0, -lin_speed, 0.0         # right strafe (vy < 0)

    # Diagonal / "turning" with both vx and vy
    if k == "z":
        # forward-left: vx > 0, vy > 0
        return lin_speed, 0.5 * lin_speed, 0.0
    if k == "c":
        # forward-right: vx > 0, vy < 0
        return lin_speed, -0.5 * lin_speed, 0.0

    # In-place rotation
    if k == "q":
        return 0.0, 0.0, rot_speed          # rotate left (CCW)
    if k == "e":
        return 0.0, 0.0, -rot_speed         # rotate right (CW)

    # Explicit stop
    if k == " " or k == "x":
        return 0.0, 0.0, 0.0

    # Any other key: ignore
    return None


class Navigation(Node):
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("Navigation Node Start!")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "mecanum_cmd_vel")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("lin_speed", 0.52)
        self.declare_parameter("rot_speed", 0.683)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.rate = float(self.get_parameter("rate").value)
        self.lin_speed = float(self.get_parameter("lin_speed").value)
        self.rot_speed = float(self.get_parameter("rot_speed").value)

        if self.rate <= 0.0:
            self.get_logger().warn("rate <= 0, clamping to 10 Hz")
            self.rate = 10.0

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer for keyboard polling
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"WASD teleop started, publishing Twist on '{self.cmd_vel_topic}'"
        )
        self.get_logger().info(
            "Controls:\n"
            "  w/s : forward/backward (vx)\n"
            "  a/d : left/right strafe (vy)\n"
            "  q/e : rotate left/right (w)\n"
            "  z   : forward-left (vx, vy)\n"
            "  c   : forward-right (vx, -vy)\n"
            "  space or x : stop\n"
            "  Ctrl+C : quit\n"
        )

    def timer_callback(self):
        key = get_key_nonblocking()
        if key is None:
            return

        cmd = key_to_cmd(key, self.lin_speed, self.rot_speed)
        if cmd is None:
            return

        vx, vy, w = cmd
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w

        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f"[KEY '{repr(key)[1:-1]}'] vx={vx:.3f}, vy={vy:.3f}, w={w:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    # Put stdin into cbreak mode so we get single keypresses without ENTER
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    node = Navigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WASD teleop (Ctrl+C).")
    finally:
        node.destroy_node()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()


if __name__ == "__main__":
    main()

