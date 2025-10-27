import rclpy
from rclpy.node import Node

class DistanceSensor(Node):
	def __init__(self):
		super().__init__('distance_sensor')
		self.get_logger().info('Distance Sensor Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = DistanceSensor()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
