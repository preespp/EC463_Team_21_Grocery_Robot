#!/usr/bin/env python3
"""
ROS2 node: I2C mecanum driver for ESP32, with obstacle safety gating.

- Subscribes to:
    * geometry_msgs/Twist on 'mecanum_cmd_vel' (configurable)
    * std_msgs/Bool on 'obstacle_alert'

- Maintains the last commanded velocities (vx, vy, w).
- Periodically sends those {vx, vy, w} over I2C as packed floats.
- If obstacle_alert is True, sends zeros regardless of commanded velocity.
- If no command is received for 'hold_timeout' seconds, forces stop.

ROS2 parameters:
  bus           (int)   : I2C bus number (default: 7)
  addr          (int)   : I2C slave address as int (default: 70 == 0x46)
  rate          (float) : I2C send rate in Hz (default: 50.0)
  hold_timeout  (float) : watchdog timeout in seconds (default: 3.0)
  cmd_vel_topic (string): incoming Twist topic (default: "mecanum_cmd_vel")
"""

import struct
import time

from smbus2 import SMBus

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class WheelMotor(Node):
    def __init__(self):
        super().__init__('wheel_motor')
        self.get_logger().info('Wheel Motor Node Start!')

        # Parameters
        self.declare_parameter("bus", 7)
        self.declare_parameter("addr", 70)          # 70 == 0x46
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("hold_timeout", 3.0)
        self.declare_parameter("cmd_vel_topic", "mecanum_cmd_vel")

        self.bus_id = self.get_parameter("bus").value
        self.addr = int(self.get_parameter("addr").value)
        self.rate = float(self.get_parameter("rate").value)
        self.hold_timeout = float(self.get_parameter("hold_timeout").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        if self.rate <= 0.0:
            self.get_logger().warn("rate <= 0, clamping to 10 Hz")
            self.rate = 10.0

        # Open I2C bus
        try:
            self.bus = SMBus(self.bus_id)
        except FileNotFoundError as exc:
            self.get_logger().fatal(
                f"Cannot open /dev/i2c-{self.bus_id}: {exc}"
            )
            raise

        # Last commanded velocities
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.last_cmd_time = time.monotonic()

        # Obstacle flag from ultrasonic node
        self.obstacle_active = False

        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.obstacle_sub = self.create_subscription(
            Bool,
            "obstacle_alert",
            self.obstacle_callback,
            10,
        )

        # Timer for periodic I2C sending + timeout handling
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"I2C mecanum driver started on /dev/i2c-{self.bus_id}, "
            f"addr 0x{self.addr:02X}"
        )
        self.get_logger().info(
            f"Subscribing to Twist on '{self.cmd_vel_topic}' "
            "and obstacle_alert on 'obstacle_alert'."
        )

    def cmd_vel_callback(self, msg: Twist):
        """Update commanded velocity from teleop (or any other cmd_vel source)."""
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.w = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

        self.get_logger().debug(
            f"[CMD_VEL] vx={self.vx:.3f}, vy={self.vy:.3f}, w={self.w:.3f}"
        )

    def obstacle_callback(self, msg: Bool):
        """Handle obstacle_alert (True = obstacle, False = clear)."""
        prev = self.obstacle_active
        self.obstacle_active = msg.data

        if msg.data and not prev:
            self.get_logger().warn(
                "Obstacle detected: suppressing motion (sending 0,0,0)."
            )
        elif not msg.data and prev:
            self.get_logger().info(
                "Obstacle cleared: motion re-enabled."
            )

    def timer_callback(self):
        """Watchdog + I2C send."""
        now = time.monotonic()

        # 1) Apply hold timeout (if moving and no new command for a while)
        if (self.vx != 0.0 or self.vy != 0.0 or self.w != 0.0) and (
            now - self.last_cmd_time > self.hold_timeout
        ):
            self.vx = self.vy = self.w = 0.0
            self.get_logger().info("[TIMEOUT] No cmd_vel recently, sending stop.")

        # 2) Decide what to send, based on obstacle flag
        if self.obstacle_active:
            send_vx = send_vy = send_w = 0.0
        else:
            send_vx, send_vy, send_w = self.vx, self.vy, self.w

        # 3) Pack and send over I2C
        packet = struct.pack("<fff", send_vx, send_vy, send_w)
        try:
            self.bus.write_i2c_block_data(self.addr, 0x00, list(packet))
        except OSError as exc:
            self.get_logger().error(f"[I2C ERROR] {exc}")

        # Optional debug:
        # self.get_logger().info(
        #     f"[TX] vx={send_vx:.3f}, vy={send_vy:.3f}, w={send_w:.3f}, "
        #     f"obstacle={self.obstacle_active}"
        # )

    def destroy_node(self):
        """Ensure I2C bus is closed on shutdown."""
        try:
            self.bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down I2C mecanum driver (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
