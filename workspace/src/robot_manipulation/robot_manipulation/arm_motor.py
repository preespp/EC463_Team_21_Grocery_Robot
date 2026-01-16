# Test I2C with 5 servo motors
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from smbus2 import SMBus
import struct

class ServoI2C(Node):
    def __init__(self):
        super().__init__("arm_motor_i2c")

        # Declare ROS parameters
        self.declare_parameter("bus", 7)
        self.declare_parameter("addr", 0x08)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("joint_count", 5)
        self.declare_parameter("topic", "/arm/joint_cmd")

        # Load parameters
        self.bus_id = self.get_parameter("bus").value
        self.addr = self.get_parameter("addr").value
        self.rate = self.get_parameter("rate").value
        self.joint_count = self.get_parameter("joint_count").value
        topic_name = self.get_parameter("topic").value

        # Open I2C bus
        try:
            self.bus = SMBus(self.bus_id)
        except FileNotFoundError:
            self.get_logger().error(f"Could not open /dev/i2c-{self.bus_id}")
            raise

        self.get_logger().info(
            f"Started I2C Servo Sender on /dev/i2c-{self.bus_id}, "
            f"addr=0x{self.addr:02X}, sending {self.joint_count} floats"
        )

        # Latest joint values
        self.joints = [0.0] * self.joint_count

        # Subscriber
        self.sub = self.create_subscription(
            Float32MultiArray,
            topic_name,
            self.joint_cmd_callback,
            10
        )

        # Timer to transmit periodically
        self.timer = self.create_timer(1.0 / self.rate, self.send_i2c)

    def joint_cmd_callback(self, msg: Float32MultiArray):
        """
        Store the first N received joint values.
        """
        count = min(self.joint_count, len(msg.data))
        for i in range(count):
            self.joints[i] = float(msg.data[i])

    def send_i2c(self):
        """
        Pack 5 floats → send to ESP32.
        """
        packet = struct.pack("<" + "f" * self.joint_count, *self.joints)

        try:
            # 0x00 register is arbitrary; ESP32 ignores it
            self.bus.write_i2c_block_data(self.addr, 0x00, list(packet))
        except Exception as e:
            self.get_logger().warn(f"I2C send failed: {e}")
            return

        self.get_logger().info(
            "I2C TX: " + ", ".join(f"{v:.2f}" for v in self.joints)
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
