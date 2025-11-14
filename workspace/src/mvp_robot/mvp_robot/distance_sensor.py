import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import serial
import json
import glob

class DistanceSensor(Node):
    def __init__(self):
        super().__init__('distance_sensor')

        self.distance_pub = self.create_publisher(Float32, 'distance_cm', 10)
        self.alert_pub = self.create_publisher(Bool, 'obstacle_alert', 10)

        self.get_logger().info('Distance Sensor Node started!')

        # # Configure UART Serial Port (via USB)
        # self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        
        # UART Port
        self.serial_port = serial.Serial('/dev/ttyTHS1', 115200, timeout=0.1)
        	
        # Timer for 20 Hz polling
        self.timer = self.create_timer(0.05, self.read_serial)
        
        self.threshold = 30.0
    
    def read_serial(self):
    	try:
    		raw = self.serial_port.readline()
    		
    		if not raw:
    			self.get_logger().warn("Empty line from serial")
    			return
    			
    		line = raw.decode('utf-8', errors='ignore').strip()
    		print(f"Serial line: {line}")
    		
    		# If there is any extra text on the line, keep only the JSON part
    		if '{' in line and '}' in line:
    			line = line[line.index('{'): line.rindex('}') + 1]
    		
    		data = json.loads(line)
    		
    		# ESP32 sends centimeters: {"distance_cm": 812.85}
    		distance_cm = float(data["distance_cm"])
    		
    		# Publish distance
    		distance_msg = Float32()
    		distance_msg.data = distance_cm
    		self.distance_pub.publish(distance_msg)
    		
    		alert_msg = Bool()
    		alert_msg.data = distance_msg.data < self.threshold
    		self.alert_pub.publish(alert_msg)
    		
    		self.get_logger().info(
    			f"Distance: {distance_msg.data:.1f} (same units as threshold) | "
    			f"Obstacle Alert: {alert_msg.data}"
    		)
    	except (json.JSONDecodeError, KeyError, ValueError) as e:
    		self.get_logger().warn(f"Skip malformed data: {e}")
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
