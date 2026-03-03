import py_trees
import rclpy
from rclpy.action import ActionClient
from rack_interfaces.action import MoveRack

class RepositionRackToGoalLevel(py_trees.behaviour.Behaviour):
    """
    - Calls ROS2 Action /move_rack
    - Returns SUCCESS only when rack motion finished
    """
    def __init__(self, node, goal_key):
        super().__init__(f"RepositionRackToGoalLevel[{goal_key}]")
        self.node = node
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()
        
        self.client = ActionClient(
            node,
            MoveRack,
            "/move_rack"
        )

        self.goal_handle = None
        self.result_future = None
        self.sent_goal = False

    def setup(self, timeout):
        if not self.client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Rack action server not available")

    def initialise(self):
        self.sent_goal = False
        self.goal_handle = None
        self.result_future = None

    def update(self):
        # Resolve level
        if isinstance(self.goal_key, str):
            level = getattr(self.bb, self.goal_key, None)
        else:
            level = self.goal_key

        if level is None:
            self.node.get_logger().error("Shelf level not found")
            return py_trees.common.Status.FAILURE

        # Send goal once
        if not self.sent_goal:
            goal_msg = MoveRack.Goal()
            goal_msg.shelf_level = int(level)

            send_future = self.client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            rclpy.spin_until_future_complete(self.node, send_future)
            self.goal_handle = send_future.result()

            if not self.goal_handle.accepted:
                self.node.get_logger().error("Rack goal rejected")
                return py_trees.common.Status.FAILURE

            self.result_future = self.goal_handle.get_result_async()
            self.sent_goal = True

            return py_trees.common.Status.RUNNING

        # Wait for result
        if self.result_future.done():
            result = self.result_future.result().result

            if result.success:
                self.node.get_logger().info("Rack motion finished")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(result.message)
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def feedback_callback(self, feedback_msg):

        state = feedback_msg.feedback.state
        height = feedback_msg.feedback.target_height_mm

        self.node.get_logger().info(
            f"[Rack] state={state} target={height:.2f} mm"
        )

    def terminate(self, new_status):
        # cancel goal if tree aborts
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
