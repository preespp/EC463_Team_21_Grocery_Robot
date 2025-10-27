import rclpy
from rclpy.node import Node

class Central(Node):
	def __init__(self):
		super().__init__('central')
		self.get_logger().info('Central Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = RobotController()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
