#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import struct
from smbus2 import SMBus, i2c_msg


class DistanceSensor(Node):
    def __init__(self):
        super().__init__('ultrasonic_i2c')

        # ---------------- Parameters ----------------
        self.declare_parameter("bus", 7)
        self.declare_parameter("addr", 9)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("threshold_m", 0.05)
        self.declare_parameter("name", "front")

        self.bus_id = int(self.get_parameter("bus").value)
        self.addr = int(self.get_parameter("addr").value)
        self.rate = float(self.get_parameter("rate").value)
        self.threshold_m = float(self.get_parameter("threshold_m").value)
        self.name = self.get_parameter("name").value

        if self.rate <= 0.0:
            self.get_logger().warn("rate <= 0, clamping to 10 Hz")
            self.rate = 10.0

        # ---------------- I2C ----------------
        try:
            self.bus = SMBus(self.bus_id)
        except FileNotFoundError as exc:
            self.get_logger().fatal(
                f"[{self.name}] Cannot open /dev/i2c-{self.bus_id}: {exc}"
            )
            raise

        # ---------------- Publisher ----------------
        self.alert_pub = self.create_publisher(
            Bool,
            f"{self.name}_alert",
            10
        )

        # ---------------- Timer ----------------
        self.timer = self.create_timer(
            1.0 / self.rate,
            self.timer_callback
        )

        self.get_logger().info(
            f"[{self.name}] Ultrasonic I2C started on "
            f"/dev/i2c-{self.bus_id}, addr 0x{self.addr:02X}, "
            f"rate {self.rate:.1f} Hz"
        )

    def timer_callback(self):
        try:
            # Read exactly 8 bytes from the slave without SMBus command byte.
            # ESP-IDF i2c slave APIs expect raw I2C transfers.
            msg = i2c_msg.read(self.addr, 8)
            self.bus.i2c_rdwr(msg)
            dist_left_m, dist_right_m = struct.unpack("<ff", bytes(msg))

            self.get_logger().info(
                f"{dist_left_m}, {dist_right_m}"
            )

            alert = (
                0 < dist_left_m < self.threshold_m or
                0 < dist_right_m < self.threshold_m
            )

            msg = Bool()
            msg.data = alert
            self.alert_pub.publish(msg)

        except OSError as e:
            self.get_logger().warn(
                f"[{self.name}] I2C read failed: {e}"
            )

        except struct.error as e:
            self.get_logger().warn(
                f"[{self.name}] Malformed packet: {e}"
            )

    def destroy_node(self):
        try:
            self.bus.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
