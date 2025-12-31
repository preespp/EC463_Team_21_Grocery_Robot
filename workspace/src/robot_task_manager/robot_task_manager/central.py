# central.py

import rclpy
from rclpy.node import Node
from mvp_robot_msgs.srv import ProductRequest


class Central(Node):
    def __init__(self):
        super().__init__('central_node')
        self.get_logger().info('Central Node Start!')

        # Service UI calls to get product info
        self.product_request_srv = self.create_service(
            ProductRequest,
            'product_request',
            self.handle_request
        )

        # Optional cache if your external DB will be expensive later
        self.product_cache = {}

        # Mock internal "database" for now
        # product_id -> (name, aisle, shelf_level)
        self.database = {
            'A123': ('Bottle', 'A', 1),
            'B1098': ('Bottle', 'B', 3),
        }

        self.get_logger().info("Central Node ready!")

    # ---- future-friendly lookup hook ----
    def lookup_product(self, product_id: str):
        """
        Lookup product info.

        For now: use internal dict.
        In the future: replace this with MongoDB / external API calls.
        """
        # 1) Cache check first (optional but useful for real DB)
        if product_id in self.product_cache:
            self.get_logger().info(f"Cache hit in lookup_product for {product_id}")
            return self.product_cache[product_id]

        # 2) Local mock DB fallback
        if product_id in self.database:
            name, aisle, shelf_level = self.database[product_id]
            location = f"{aisle}{shelf_level}"  # e.g., "A1"

            product = {
                'found': True,
                'name': name,
                'aisle': aisle,
                'shelf_level': shelf_level,
                'location': location,
            }

            # store in cache for future
            self.product_cache[product_id] = product
            return product

        # Not found
        return {
            'found': False,
            'name': '',
            'aisle': '',
            'shelf_level': 0,
            'location': '',
        }

    # ---- service callback ----
    def handle_request(self, request, response):
        product_id = request.product_id
        self.get_logger().info(f"[Central] Received request for product_id={product_id}")

        product = self.lookup_product(product_id)

        if product['found']:
            response.found = True
            response.product_name = product['name']
            response.product_location = product['location']
            self.get_logger().info(
                f"[Central] Product {product_id} -> "
                f"{product['name']} at {product['location']}"
            )

            # --- future hooks for robot ---
            # Here is where you will later:
            # - call move_rack service
            # - call move_gripper service
            # - trigger SLAM / navigation, etc.
            #
            # For example, later:
            # self.send_pick_task(product_id, product['aisle'], product['shelf_level'])

        else:
            response.found = False
            response.product_name = ""
            response.product_location = ""
            self.get_logger().warn(f"[Central] Product {product_id} not found.")

        return response

    # Example placeholder for future robot orchestration
    # def send_pick_task(self, product_id, aisle, shelf_level):
    #     # This will call other services/actions:
    #     # - /move_base or Nav2 action
    #     # - /move_rack service
    #     # - /move_gripper service
    #     # For now, we just log.
    #     self.get_logger().info(
    #         f"[Central] (TODO) send pick task: {product_id} at aisle {aisle}, shelf {shelf_level}"
    #     )


def main(args=None):
    rclpy.init(args=args)
    node = Central()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

