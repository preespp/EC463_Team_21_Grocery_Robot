import rclpy
from rclpy.node import Node
from mvp_interfaces.srv import ProdcutRequest.srv

class UiInput(Node):
	def __init__(self):
		super().__init__('ui_input')
		self.get_logger().info('UI Input Node Start!')
		self.client = self.create_client(ProductRequest, 'product_request')
	
	def send_request(self, product_id):
		if not self.client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error("Central service unavailable")
			return
		
		request = ProductRequest.Request()
		request.product_id = product_id
		future = self.client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		response = future.result()
		
		if response and response.success:
			resp_msg = f"Product {product_id}: {response.product_name} is "
			resp_msg += "found" if response.found else "not found"
			if response.found:
				resp_msg += f"{ located at {response.product_location}"
				self.get_logger().info(resp_msg)
			else:
				self.get_logger().warn(resp_msg)
		
def main(args=None):
	rclpy.init(args=args)
	node = UiInput()
	node.send_request("A123") # product 1
	node.send_request("B1098") # product 2
	node.send_request("C1012") # # not found product
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
