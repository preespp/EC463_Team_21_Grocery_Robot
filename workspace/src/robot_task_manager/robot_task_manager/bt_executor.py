import rclpy
from rclpy.node import Node
import requests
import py_trees

from robot_interfaces.srv import NewOrder
from robot_interfaces.msg import Order, OrderItem

from robot_task_manager.blackboard import setup_blackboard
from robot_task_manager.trees.customer import create_customer_tree
from robot_task_manager.trees.restock import create_restock_tree

SERVER = "http://localhost:3000"


class BTExecutor(Node):
    def __init__(self):
        super().__init__("bt_executor")

        self.bb = setup_blackboard()

        self.current_order: Order | None = None
        self.robot_busy = False

        self.create_service(NewOrder, "/order/new", self.new_order_cb)

        self.poll_timer = self.create_timer(1.0, self.nodejs)

        self.tree: py_trees.trees.BehaviourTree | None = None
        self.bt_tick_timer = self.create_timer(0.1, self.tick_bt)

        self.get_logger().info("BTExecutor ready (order intake + blackboard + BT)")

    def new_order_cb(self, request, response):
        if self.current_order is not None:
            response.accepted = False
            response.message = "Order already loaded"
            return response

        self.accept_order(request.order)
        response.accepted = True
        response.message = "Order loaded"
        return response

    def nodejs(self):
        if self.robot_busy or self.current_order is not None:
            return

        try:
            r = requests.get(f"{SERVER}/api/order/latest", timeout=1.0)
            if r.status_code != 200:
                return

            data = r.json()
            order_msg = self.convert_json_to_order(data)
            self.accept_order(order_msg)

            requests.post(f"{SERVER}/api/order/ack", timeout=1.0)

        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Node.js unreachable: {e}")

        except Exception as e:
            self.get_logger().error(f"BT build error: {e}")
            self.clear_order()

    def accept_order(self, order: Order):
        self.current_order = order
        self.robot_busy = True

        # Put everything into blackboard
        self.bb.order = order
        self.bb.mode = str(order.role).strip().lower()
        self.bb.items = list(order.items)
        self.bb.item_index = 0
        self.bb.current_item = None

        self.get_logger().info(
            f"Order loaded | id={order.order_id} | role={self.bb.mode} | items={len(self.bb.items)}"
        )

        self.build_tree_from_mode()

    def clear_order(self):
        self.current_order = None
        self.robot_busy = False

        # Clear order data
        self.tree = None
        self.bb.order = None
        self.bb.items = []
        self.bb.item_index = 0
        self.bb.current_item = None
        self.bb.num_current_item = 0
        self.bb.mode = None
        self.bb.nav_goal = (0.0, 0.0)
        self.bb.rack_goal = 1
        self.bb.shelf_pose = None
        self.bb.current_rack = 1
        self.bb.pose = None

        self.get_logger().info("Order cleared -> IDLE")

    def build_tree_from_mode(self):
        if self.tree is not None:
            self.get_logger().warn("BT already running; ignoring build request")
            return

        mode = self.bb.mode
        if mode not in ("customer", "employee"):
            self.get_logger().error(
                f"Unknown role/mode: {mode}. Expected 'customer' or 'employee'."
            )
            self.clear_order()
            return

        if mode == "customer":
            root = create_customer_tree(self.bb)
        else:
            root = create_restock_tree(self.bb)

        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(timeout=5)
        self.get_logger().info(f"BT started in mode={mode}")

    def tick_bt(self):
        if self.tree is None:
            return

        self.tree.tick()

        status = self.tree.root.status
        self.get_logger().info(f"BT status: {status}")

        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
            self.get_logger().info(f"BT finished: {status}")
            try:
                if self.bb.order is not None:
                    requests.post(
                        f"{SERVER}/api/order/complete",
                        json={
                            "order_id": int(self.bb.order.order_id),
                            "result": str(status),
                        },
                        timeout=1.0,
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to report completion: {e}")

            self.clear_order()

    def convert_json_to_order(self, data: dict) -> Order:
        order = Order()
        order.order_id = int(data["order_id"])
        order.role = data["role"]
        order.requester_id = str(data.get("requester_id", data.get("id", "")))

        for it in data["items"]:
            item = OrderItem()
            item.product_id = it["product_id"]
            item.name = it["name"]
            item.aisle = it["aisle"]
            item.rack = int(it["rack"])
            item.shelf_level = int(it["shelf_level"])
            item.qty = int(it["qty"])
            item.price = float(it["price"])
            item.stock = int(it["stock"])
            order.items.append(item)

        return order


def main():
    rclpy.init()
    node = BTExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

