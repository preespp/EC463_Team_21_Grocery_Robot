import rclpy
from rclpy.node import Node

class UiInput(Node):
	def __init__(self):
		super().__init__('ui_input')
		self.get_logger().info('UI Input Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = UiInput()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
