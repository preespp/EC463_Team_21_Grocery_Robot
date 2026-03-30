import py_trees
import rclpy
from rclpy.action import ActionClient
from robot_interfaces.action import MoveRack

class RepositionRackToGoalLevel(py_trees.behaviour.Behaviour):
    """
    - Calls ROS2 Action /move_rack
    - Returns SUCCESS only when rack motion finished
    """
    def __init__(self, goal_key: str, action_name: str = "/move_rack"):
        super().__init__(f"RepositionRackToGoalLevel[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()

        self.action_name = action_name
        self.node = None
        self.client = None

        self.goal_handle = None
        self.result_future = None
        self.sent_goal = False

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_rack_goal_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, MoveRack, self.action_name)
        return True

    def initialise(self):
        self.sent_goal = False
        self.goal_handle = None
        self.result_future = None

    def update(self):
        # Resolve level
        level = getattr(self.bb, self.goal_key, None)
        current_level = getattr(self.bb, "current_rack", None)
        if level is None or current_level is None:
            self.feedback_message = f"Missing rack level: target={level}, current={current_level}"
            return py_trees.common.Status.FAILURE

        diff_level = level - current_level

        if diff_level == 0:
            self.feedback_message = "Already at target rack level"
            return py_trees.common.Status.SUCCESS

        # Send goal once
        if not self.sent_goal:
            if not self.client.wait_for_server(timeout_sec=2.0):
                self.feedback_message = f"Rack action unavailable: {self.action_name}"
                return py_trees.common.Status.FAILURE

            goal_msg = MoveRack.Goal()
            goal_msg.shelf_level = int(diff_level)

            send_future = self.client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )

            rclpy.spin_until_future_complete(self.node, send_future)
            self.goal_handle = send_future.result()

            if not self.goal_handle.accepted:
                self.feedback_message = "Rack goal rejected"
                return py_trees.common.Status.FAILURE

            self.result_future = self.goal_handle.get_result_async()
            self.sent_goal = True
            self.feedback_message = f"Moving rack from {current_level} to {level}"

            return py_trees.common.Status.RUNNING

        # Wait for result
        rclpy.spin_once(self.node, timeout_sec=0.05)
        if self.result_future.done():
            result = self.result_future.result().result

            if result.success:
                self.bb.current_rack = level
                self.feedback_message = "Rack motion finished"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = result.message
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def feedback_callback(self, feedback_msg):

        state = feedback_msg.feedback.state
        height = feedback_msg.feedback.target_height_mm

        self.feedback_message = f"Rack {state} target={height:.2f} mm"

    def terminate(self, new_status):
        # cancel goal if tree aborts
        if new_status == py_trees.common.Status.INVALID:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.result_future = None
        self.sent_goal = False
