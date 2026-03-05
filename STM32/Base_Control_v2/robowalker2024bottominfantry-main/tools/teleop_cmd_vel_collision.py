#!/usr/bin/env python3
"""
Minimal keyboard teleop that publishes /cmd_vel for the RoboWalker stack.
W/S: forward/back, A/D: left/right strafe, Q/E: CCW/CW yaw.
Space or X: stop. Ctrl+C: quit.
"""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


KEY_BINDINGS = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}


def get_key(timeout: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        # Drain escape sequences (arrow keys, etc.)
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
        return ""
    return ch


def main() -> int:
    parser = argparse.ArgumentParser(description="Keyboard teleop publisher for /cmd_vel")
    parser.add_argument("--topic", default="/cmd_vel", help="cmd_vel topic name")
    parser.add_argument("--linear", type=float, default=0.5, help="Linear speed (m/s)")
    parser.add_argument("--angular", type=float, default=1.0, help="Angular speed (rad/s)")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate (Hz)")
    parser.add_argument("--deadman", type=float, default=0.3, help="Stop after no key for N seconds (0 to disable)")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("teleop_cmd_vel")
    pub = node.create_publisher(Twist, args.topic, 10)
    right_alert = False

    def on_right_alert(msg: Bool) -> None:
        nonlocal right_alert
        right_alert = bool(msg.data)

    node.create_subscription(Bool, "right_alert", on_right_alert, 10)

    settings = termios.tcgetattr(sys.stdin)
    last_cmd_time = time.monotonic()
    current = (0.0, 0.0, 0.0)
    period = 1.0 / max(1e-3, args.rate)

    print("Teleop ready: W/S/A/D/Q/E, Space/X to stop, Ctrl+C to quit.")
    print(f"Publishing to {args.topic} at {args.rate:.1f} Hz.")
    print(f"Speeds: linear={args.linear:.2f} m/s, angular={args.angular:.2f} rad/s.")
    print("Collision stop topic: right_alert")

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = get_key(period)
            if key:
                if key in KEY_BINDINGS:
                    current = KEY_BINDINGS[key]
                    last_cmd_time = time.monotonic()
                elif key in (" ", "x", "X"):
                    current = (0.0, 0.0, 0.0)
                    last_cmd_time = time.monotonic()
                elif key == "\x03":
                    break

            rclpy.spin_once(node, timeout_sec=0.0)

            if args.deadman > 0.0 and (time.monotonic() - last_cmd_time) > args.deadman:
                current = (0.0, 0.0, 0.0)

            if right_alert:
                current = (0.0, 0.0, 0.0)

            twist = Twist()
            twist.linear.x = current[0] * args.linear
            twist.linear.y = current[1] * args.linear
            twist.angular.z = current[2] * args.angular
            pub.publish(twist)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop = Twist()
        pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
