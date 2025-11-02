import rclpy
from rclpy.node import Node
from mvp_interfaces.srv import ProductInfo

class ProductDbQuery(Node):
	def __init__(self):
		super().__init__('product_db_query')
		self.get_logger().info('Product Database Query Node Start!')
		self.srv = self.create_service(ProductQuery, 'product_query', self.query_callback)
		# mock database to be replaced with API for real database
		self.database = { # name, aisle, shelf level
			'A123' : ('Bottle', 'A1' , 1)
			'B1098' : ('Bottle', 'B2' , 3)
		}
	
	def query_callback(self, request, response):
		product_id = request.product_id
		if product_id in self.database:
			response.found = True
			response.product_name, response.aisle, response.shelf_level = self.database[product_id]
			self.get_logger().info('Product Found!')
		else:
			response.found = False
			response.product_name = ""
			response.aisle = ""
			response.shelf_level = 1
			self.get_logger().warn('Product not found!')
		retuirn response
		
def main(args=None):
	rcl.py.init(args=args)
	node = ProductDbQuery()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
