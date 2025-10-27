import rclpy
from rclpy.node import Node

class WheelMotor(Node):
	def __init__(self):
		super().__init__('central')
		self.get_logger().info('Wheel Motor Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = WheelMotor()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
