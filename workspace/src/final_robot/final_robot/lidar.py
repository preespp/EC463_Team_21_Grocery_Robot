import rclpy
from rclpy.node import Node

class Lidar(Node):
	def __init__(self):
		super().__init__('lidar')
		self.get_logger().info('Lidar Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = Lidar()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
