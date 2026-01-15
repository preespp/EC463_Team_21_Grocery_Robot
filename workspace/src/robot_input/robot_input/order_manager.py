import rclpy
from rclpy.node import Node
import requests
from robot_interfaces.srv import NewOrder
from robot_interfaces.msg import Order, OrderItem

server = "http://localhost:3000"


class OrderManager(Node):

    def __init__(self):
        super().__init__("order_manager")

        self.robot_busy = False
        self.current_order: Order | None = None

        self.srv = self.create_service(
            NewOrder,
            "/order/new",
            self.new_order
        )

        self.timer = self.create_timer(1.0, self.poll_nodejs)

        self.get_logger().info("OrderManager ready")

    def poll_nodejs(self):
        if self.robot_busy or self.current_order is not None:
            return

        try:
            r = requests.get(f"{server}/api/order/latest", timeout=1.0)
            if r.status_code != 200:
                return

            data = r.json()
            order_msg = self.convert_json_to_order(data)
            self.accept_order(order_msg)

            # ACK Node.js
            requests.post(f"{server}/api/order/ack", timeout=1.0)

        except Exception as e:
            self.get_logger().warn(f"Node.js unreachable: {e}")

    def new_order(self, request, response):
        if self.current_order is not None:
            response.accepted = False
            response.message = "Order already loaded"
            return response

        self.accept_order(request.order)

        response.accepted = True
        response.message = "Order loaded"
        return response

    def accept_order(self, order: Order):
        self.current_order = order

        self.get_logger().info(
            f"Order loaded | ID={order.order_id} | role={order.role} | requester={order.requester_id}"
        )

        for i, item in enumerate(order.items):
            self.get_logger().info(
                f"Item {i}: {item.name} | aisle={item.aisle} | rack={item.rack} | "
                f"shelf={item.shelf_level} | qty={item.qty}"
            )

    def convert_json_to_order(self, data: dict) -> Order:
        order = Order()
        order.order_id = int(data["order_id"])
        order.role = data["role"]
        order.requester_id = str(data["id"])

        for it in data["items"]:
            item = OrderItem()
            item.name = it["name"]
            item.aisle = it["aisle"]
            item.rack = int(it["rack"])
            item.shelf_level = int(it["shelf_level"])
            item.qty = int(it["qty"])
            item.price = float(it["price"])
            item.stock = int(it["stock"])

            order.items.append(item)

        return order

    def mark_busy(self):
        self.robot_busy = True

    def clear_order(self):
        self.robot_busy = False
        self.current_order = None
        self.get_logger().info("Order cleared")


def main():
    rclpy.init()
    node = OrderManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
