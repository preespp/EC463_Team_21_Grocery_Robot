import rclpy
from rclpy.node import Node

class CameraVision(Node):
	def __init__(self):
		super().__init__('camera_vision')
		self.get_logger().info('Camera Vision Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = CameraVision()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
