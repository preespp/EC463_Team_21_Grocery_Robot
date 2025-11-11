import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial
import json

class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')

        self.distance_pub = self.create_publisher(Float32, 'distance_cm', 10)
        self.alert_pub = self.create_publisher(Bool, 'obstacle_alert', 10)

        self.get_logger().info('Distance Sensor Node started!')

        # Configure UART Serial Port
        self.serial_port = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)

        # Timer for 20 Hz polling
        self.timer = self.create_timer(0.05, self.read_serial)

        self.threshold = 30.0

    def read_serial(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                return
            data = json.loads(line)
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = float(data['distance_cm'])
            self.distance_pub.publish(distance_msg)
            # Publish obstacle alert
            alert_msg = Bool()
            alert_msg.data = distance_msg.data < self.threshold
            self.alert_pub.publish(alert_msg)
            # Log for debug
            self.get_logger().info(
                f"Distance: {distance_msg.data:.1f} cm | Obstacle Alert: {alert_msg.data}"
            )

        except (json.JSONDecodeError, KeyError, ValueError):
            # Skip malformed data
            pass
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
