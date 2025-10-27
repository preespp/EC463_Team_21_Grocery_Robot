import rclpy
from rclpy.node import Node

class IrSensor(Node):
	def __init__(self):
		super().__init__('ir_sensor')
		self.get_logger().info('IR Sensor Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = IrSensor()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
