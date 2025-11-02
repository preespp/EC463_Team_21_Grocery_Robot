import rclpy
from rclpy.node import Node
from mvp_interfaces.srv import ProductInfo
from mvp_interfaces.srv import ProductRequest
from std_msgs.msg import String

class Central(Node):
	def __init__(self):
		super().__init__('central')
		self.get_logger().info('Central Node Start!')
		self.
def main(args=None):
	rcl.py.init(args=args)
	node = Central()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
