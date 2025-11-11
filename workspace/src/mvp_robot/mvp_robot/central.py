import rclpy
from rclpy.node import Node
from mvp_robot.srv import ProductInfo, ProductRequest

class Central(Node):
	def __init__(self):
		super().__init__('central')
		self.get_logger().info('Central Node Start!')
		self.product_request_srv = self.create_service(ProductRequest, 'product_request', self.handle_request)
		self.product_query_client = self.create_client(ProductInfo, 'product_query')
		# store results for reuse
		self.product_cache = {}
		self.get_logger().info("Central Node ready!")
	
	def handle_request(self, request, response):
		product_id = request.product_id
		self.get_logger().info(f"Received request for {product_id}")

		# Check cache first
		if product_id in self.product_cache:
			self.get_logger().info(f"Cache hit for {product_id}")
			product = self.product_cache[product_id]
			response.found = True
			response.product_name = product['name']
			response.product_location = product['location']
			return response

		# Product Query Service
		if not self.product_query_client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error("Product Query service unavailable.")
			response.found = False
			return response

		query_req = ProductInfo.Request()
		query_req.product_id = product_id
		future = self.product_query_client.call_async(query_req)
		rclpy.spin_until_future_complete(self, future)
		result = future.result()

		if result is not None and result.found:
			self.product_cache[product_id] = {
				'name': result.product_name,
				'location': result.aisle + str(result.shelf_level)
		    }
			response.found = True
			response.product_name = result.product_name
			response.product_location = result.location
			self.get_logger().info(f"Stored {product_id} in cache.")
		else:
			response.found = False
			response.product_name = ""
			response.product_location = ""

		return response

def main():
	rclpy.init()
	node = Central()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
