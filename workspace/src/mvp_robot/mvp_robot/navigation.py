import rclpy
from rclpy.node import Node

class Navigation(Node):
	def __init__(self):
		super().__init__('navigation')
		self.get_logger().info('Navigation Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = Navigation()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
