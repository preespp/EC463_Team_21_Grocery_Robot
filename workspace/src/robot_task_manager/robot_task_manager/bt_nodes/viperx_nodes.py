import py_trees
import rclpy
from rclpy.action import ActionClient
from interbotix_xsarm_msgs.action import MoveItJointTrajectory
from interbotix_xsarm_msgs.srv import SetGripper
import requests

class RepositionViperXArm(py_trees.behaviour.Behaviour):
    """
    Reads a pose from bb.<goal_key> and commands the ViperX arm via MoveIt action.
    """

    def __init__(self, goal_key: str):
        super().__init__(f"RepositionViperXArm[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()
        self.client: ActionClient | None = None
        self.future = None
        self.node: rclpy.node.Node | None = None

    def setup(self, timeout):
        self.node = rclpy.get_node("bt_executor")
        self.client = ActionClient(self.node, MoveItJointTrajectory, "viperx/arm/moveit_trajectory")
        return True

    def initialise(self):
        target_pose = getattr(self.bb, self.goal_key, None)
        if target_pose is None:
            self.feedback_message = f"No pose found for {self.goal_key}"
            return

        goal_msg = MoveItJointTrajectory.Goal()
        goal_msg.joint_positions = target_pose["joint_positions"]
        self.future = self.client.send_goal_async(goal_msg)
        self.feedback_message = f"Sending waypoint to ViperX: {self.goal_key}"

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            result = self.future.result().result
            self.feedback_message = f"ViperX reached {self.goal_key}"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class VerifyViperXPosition(py_trees.behaviour.Behaviour):
    """
    Placeholder for vision/pose verification.
    Could later call a vision service or IK validation node.
    """

    def __init__(self):
        super().__init__("VerifyViperXPosition")
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        # TODO: implement camera or sensor feedback
        # For now, we just mark success
        self.bb.pose = None  # Could be updated after IK from camera frame
        return py_trees.common.Status.SUCCESS


class MoveViperXGripper(py_trees.behaviour.Behaviour):
    """
    Command the ViperX gripper: "open" or "close"
    """

    def __init__(self, command: str):
        super().__init__(f"MoveViperXGripper[{command}]")
        self.command = command
        self.bb = py_trees.blackboard.Blackboard()
        self.client = None
        self.future = None
        self.node: rclpy.node.Node | None = None

    def setup(self, timeout):
        self.node = rclpy.get_node("bt_executor_viperx")
        self.client = self.node.create_client(SetGripper, "viperx/gripper/move")
        return True

    def initialise(self):
        if self.command not in ("open", "close"):
            self.feedback_message = f"Invalid gripper command: {self.command}"
            return

        req = SetGripper.Request()
        req.position = 0.0 if self.command == "open" else 1.0
        self.future = self.client.call_async(req)
        self.feedback_message = f"Gripper command sent: {self.command}"

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE
        if self.future.done():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING
