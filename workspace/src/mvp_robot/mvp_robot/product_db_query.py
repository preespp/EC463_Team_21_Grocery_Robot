import rclpy
from rclpy.node import Node

class ProductDbQuery(Node):
	def __init__(self):
		super().__init__('product_db_query')
		self.get_logger().info('Product Database Query Node Start!')
		
def main(args=None):
	rcl.py.init(args=args)
	node = ProductDbQuery()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
