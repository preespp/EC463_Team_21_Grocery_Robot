# ui_input.py

import rclpy
from rclpy.node import Node
from mvp_robot_msgs.srv import ProductRequest


class UiInput(Node):
    def __init__(self):
        super().__init__('ui_input')
        self.get_logger().info('UI Input Node Started!')
        self.client = self.create_client(ProductRequest, 'product_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for central service...')

        self.get_logger().info('Central service available. Ready to take input.')

    def send_request(self, product_id: str):
        request = ProductRequest.Request()
        request.product_id = product_id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response is None:
            self.get_logger().error("Service call returned no result.")
            return

        if response.found:
            self.get_logger().info(
                f"Product {product_id}: {response.product_name} "
                f"found at {response.product_location}"
            )
        else:
            self.get_logger().warn(f"Product {product_id} not found.")


def main(args=None):
    rclpy.init(args=args)
    node = UiInput()

    try:
        while rclpy.ok():
            product_id = input("Enter product ID (or 'q' to quit): ").strip()
            if product_id.lower() == 'q':
                break
            if product_id:
                node.send_request(product_id)
    except KeyboardInterrupt:
        print("\nShutting down UiInput node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

