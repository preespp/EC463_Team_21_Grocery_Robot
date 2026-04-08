import rclpy
from rclpy.node import Node
import requests
import py_trees
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from robot_interfaces.srv import NewOrder
from robot_interfaces.msg import Order, OrderItem

from robot_task_manager.blackboard import reset_viperx_manipulation_state, setup_blackboard
from robot_task_manager.trees.customer_viperx import create_customer_viperx_tree
from robot_task_manager.trees.restock_viperx import create_restock_viperx_tree

SERVER = "http://localhost:3000"


class BTExecutor(Node):
    def __init__(self):
        super().__init__("bt_executor")

        self.bb = setup_blackboard()
        self.declare_parameter("skip_navigation", False)
        self.declare_parameter("detection_min_confidence", 0.40)
        self.declare_parameter("return_home_retry_attempts", 3)
        self.bb.skip_navigation = bool(self.get_parameter("skip_navigation").value)
        self.bb.viperx_detection_min_confidence = float(
            self.get_parameter("detection_min_confidence").value
        )
        self.bb.viperx_return_home_retry_attempts = int(
            self.get_parameter("return_home_retry_attempts").value
        )

        self.current_order: Order | None = None
        self.robot_busy = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bb.shared_tf_buffer = self.tf_buffer

        self.create_service(NewOrder, "/order/new", self.new_order_cb)
        self.create_subscription(String, "/detections_json", self._on_detections, 10)

        self.poll_timer = self.create_timer(1.0, self.nodejs)

        self.tree: py_trees.trees.BehaviourTree | None = None
        self.bt_tick_timer = self.create_timer(0.1, self.tick_bt)

        self.get_logger().info(
            "BTExecutor ready (order intake + blackboard + BT) "
            f"| skip_navigation={self.bb.skip_navigation} "
            f"| detection_min_confidence={self.bb.viperx_detection_min_confidence:.2f} "
            f"| return_home_retry_attempts={self.bb.viperx_return_home_retry_attempts}"
        )

    def _on_detections(self, msg: String):
        self.bb.latest_detections_json = msg.data
        self.bb.latest_detection_time = self.get_clock().now()

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
            self.bb.order_id_text = str(
                data.get("order_id")
                or data.get("restock_id")
                or data.get("id", "")
            )
            self.accept_order(order_msg)

            requests.post(
                f"{SERVER}/api/order/ack",
                json={"order_id": self.bb.order_id_text},
                timeout=1.0
            )

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
        if not getattr(self.bb, "order_id_text", None):
            self.bb.order_id_text = str(order.order_id)
        self.bb.mode = str(order.role).strip().lower()
        self.bb.items = list(order.items)
        self.bb.item_index = 0
        self.bb.current_item = None
        self.bb.customer_order_items_completed = False
        reset_viperx_manipulation_state(self.bb)

        self.get_logger().info(
            f"Order loaded | id={order.order_id} | role={self.bb.mode} | items={len(self.bb.items)}"
        )

        self.build_tree_from_mode()

    def clear_order(self):
        self.current_order = None
        self.robot_busy = False

        # Clear order data
        if self.tree is not None:
            try:
                self.tree.shutdown()
            except Exception as e:
                self.get_logger().warn(f"BT shutdown warning: {e}")
        self.tree = None
        self.bb.order = None
        self.bb.order_id_text = None
        self.bb.items = []
        self.bb.item_index = 0
        self.bb.current_item = None
        self.bb.num_current_item = 0
        self.bb.customer_order_items_completed = False
        self.bb.mode = None
        self.bb.nav_goal = (0.0, 0.0, 0.0)
        self.bb.home_goal = (0.0, 0.0, 0.0)
        self.bb.rack_goal = 1
        self.bb.current_rack = 1
        self.bb.home_rack = 1
        self.bb.slot_id = None
        self.bb.anchor_id = None
        self.bb.rack_id = None
        self.bb.semantic_id = None
        self.bb.semantic_target_label = None
        self.bb.nav_goal_source = None
        reset_viperx_manipulation_state(self.bb)

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
            root = create_customer_viperx_tree(self.bb)
        else:
            root = create_restock_viperx_tree(self.bb)

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
                    order_id_to_report = (
                        self.bb.order_id_text
                        if getattr(self.bb, "order_id_text", None)
                        else str(self.bb.order.order_id)
                    )
                    requests.post(
                        f"{SERVER}/api/order/complete",
                        json={
                            "order_id": order_id_to_report,
                            "result": str(status),
                        },
                        timeout=1.0,
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to report completion: {e}")

            self.clear_order()

    def convert_json_to_order(self, data: dict) -> Order:
        order = Order()
        order_id_raw = str(data.get("order_id", "0"))
        order.order_id = self._parse_order_numeric(order_id_raw)
        order.role = data.get("role", "customer")
        order.requester_id = str(data.get("requester_id", data.get("id", "")))

        for it in data.get("items", []):
            item = OrderItem()
            x = it.get("x", it.get("aisle", 0))
            y = it.get("y", it.get("rack", 0))
            z = it.get("z", it.get("shelf_level", 0))
            item.product_id = str(it.get("product_id", ""))
            item.name = str(it.get("name", ""))
            item.aisle = str(x)
            item.rack = int(float(y))
            item.shelf_level = int(float(z))
            item.qty = int(it.get("qty", 0))
            item.price = float(it.get("price", 0.0))
            item.stock = int(it.get("stock", 0))
            order.items.append(item)

        return order

    @staticmethod
    def _parse_order_numeric(order_id_raw: str) -> int:
        # ROS message field is int64; keep compatibility with string IDs like AAA001O12345.
        digits = "".join(ch for ch in str(order_id_raw) if ch.isdigit())
        return int(digits) if digits else 0


def main():
    rclpy.init()
    node = BTExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
