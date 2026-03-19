import json
from copy import deepcopy
from typing import Optional

import py_trees
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from robot_interfaces.action import PickArm
from std_msgs.msg import String


class RepositionViperXArm(py_trees.behaviour.Behaviour):
    """
    Read a target pose from blackboard and send a PickArm action goal to ViperX.
    The target may be:
      - geometry_msgs.msg.PoseStamped
      - dict with keys frame_id, x, y, z, qx, qy, qz, qw
      - dict with keys frame_id, position{x,y,z}, orientation{x,y,z,w}
    """

    def __init__(self, goal_key: str, action_name: str = "/pick_viperx"):
        super().__init__(f"RepositionViperXArm[{goal_key}]")
        self.goal_key = goal_key
        self.action_name = action_name
        self.bb = py_trees.blackboard.Blackboard()

        self.node = None
        self.client: Optional[ActionClient] = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_viperx_arm_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, PickArm, self.action_name)
        return True

    @staticmethod
    def _dict_to_pose_stamped(target_pose: dict) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = str(target_pose.get("frame_id", "vx300/base_link"))
        pose_msg.pose.position.x = float(
            target_pose.get("x", target_pose.get("position", {}).get("x", 0.0))
        )
        pose_msg.pose.position.y = float(
            target_pose.get("y", target_pose.get("position", {}).get("y", 0.0))
        )
        pose_msg.pose.position.z = float(
            target_pose.get("z", target_pose.get("position", {}).get("z", 0.0))
        )
        pose_msg.pose.orientation.x = float(
            target_pose.get("qx", target_pose.get("orientation", {}).get("x", 0.0))
        )
        pose_msg.pose.orientation.y = float(
            target_pose.get("qy", target_pose.get("orientation", {}).get("y", 0.0))
        )
        pose_msg.pose.orientation.z = float(
            target_pose.get("qz", target_pose.get("orientation", {}).get("z", 0.0))
        )
        pose_msg.pose.orientation.w = float(
            target_pose.get("qw", target_pose.get("orientation", {}).get("w", 1.0))
        )
        return pose_msg

    def initialise(self):
        target_pose = getattr(self.bb, self.goal_key, None)
        if target_pose is None:
            self.feedback_message = f"No pose found for bb.{self.goal_key}"
            self.send_future = None
            return

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.feedback_message = f"Action server unavailable: {self.action_name}"
            self.send_future = None
            return

        goal_msg = PickArm.Goal()
        goal_msg.planning_group = "arm"
        goal_msg.ee_link = ""
        goal_msg.pregrasp_offset_m = 0.0
        goal_msg.retreat_offset_m = 0.0
        goal_msg.gripper_close_position = -1.0
        goal_msg.use_cartesian_approach = False

        if isinstance(target_pose, PoseStamped):
            goal_msg.target_pose = target_pose
        elif isinstance(target_pose, dict):
            goal_msg.target_pose = self._dict_to_pose_stamped(target_pose)
        else:
            self.feedback_message = f"Unsupported pose type for bb.{self.goal_key}"
            self.send_future = None
            return

        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.send_future = self.client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = f"Sent ViperX goal from bb.{self.goal_key}"

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE

        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.goal_handle is None and self.send_future.done():
            self.goal_handle = self.send_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.feedback_message = "ViperX goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            self.feedback_message = "ViperX goal accepted"
            return py_trees.common.Status.RUNNING

        if self.result_future is not None and self.result_future.done():
            wrapped = self.result_future.result()
            if wrapped is None:
                self.feedback_message = "ViperX goal result missing"
                return py_trees.common.Status.FAILURE
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = f"ViperX reached bb.{self.goal_key}"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"ViperX failed with status={wrapped.status}"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.client = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None


class VerifyViperXPosition(py_trees.behaviour.Behaviour):
    """
    Verify object distance using camera detections.
    If nearest object distance > threshold, update bb.pose with a corrected goal pose.

    Note:
      camera_vision publishes object points in the camera optical frame. Without TF usage here,
      this node applies a best-effort scalar correction by shifting goal x in its own frame.
    """

    def __init__(
        self,
        camera_topic: str = "/detections_json",
        max_distance_m: float = 0.22,
        max_step_m: float = 0.08,
        fallback_goal_key: str = "pose",
    ):
        super().__init__("VerifyViperXPosition")
        self.bb = py_trees.blackboard.Blackboard()
        self.camera_topic = camera_topic
        self.max_distance_m = max_distance_m
        self.max_step_m = max_step_m
        self.fallback_goal_key = fallback_goal_key

        self.node = None
        self.last_json_payload = None
        self.sub = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_viperx_verify_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.sub = self.node.create_subscription(
            String,
            self.camera_topic,
            self._on_detections,
            10,
        )
        return True

    def _on_detections(self, msg: String):
        self.last_json_payload = msg.data

    def _closest_distance(self):
        if not self.last_json_payload:
            return None
        try:
            payload = json.loads(self.last_json_payload)
            detections = payload.get("detections", [])
            distances = [
                float(det.get("distance_m", 0.0))
                for det in detections
                if float(det.get("distance_m", 0.0)) > 0.0
            ]
            if not distances:
                return None
            return min(distances)
        except Exception:
            return None

    def _update_blackboard_goal(self, delta_m: float):
        source_pose = getattr(self.bb, "pose", None)
        if source_pose is None:
            source_pose = getattr(self.bb, self.fallback_goal_key, None)
        if not isinstance(source_pose, dict):
            return False

        new_pose = deepcopy(source_pose)
        if "position" in new_pose and isinstance(new_pose["position"], dict):
            new_pose["position"]["x"] = float(new_pose["position"].get("x", 0.0)) + delta_m
        elif "x" in new_pose:
            new_pose["x"] = float(new_pose.get("x", 0.0)) + delta_m
        else:
            return False

        self.bb.pose = new_pose
        self.feedback_message = f"Adjusted bb.pose by +{delta_m:.3f} m (x-axis heuristic)"
        return True

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)

        nearest = self._closest_distance()
        if nearest is None:
            self.feedback_message = "No camera detection yet; skipping distance verify"
            return py_trees.common.Status.SUCCESS

        if nearest <= self.max_distance_m:
            self.feedback_message = (
                f"Object in range: {nearest:.3f} m <= {self.max_distance_m:.3f} m"
            )
            return py_trees.common.Status.SUCCESS

        delta = min(nearest - self.max_distance_m, self.max_step_m)
        updated = self._update_blackboard_goal(delta)
        if updated:
            return py_trees.common.Status.FAILURE

        self.feedback_message = (
            "Object too far and no mutable bb pose available for correction"
        )
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        del new_status
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.sub = None
        self.last_json_payload = None


class MoveViperXGripper(py_trees.behaviour.Behaviour):
    """
    Command ViperX gripper through PickArm action:
      - planning_group = open_gripper / close_gripper
    """

    def __init__(self, command: str, action_name: str = "/pick_viperx"):
        super().__init__(f"MoveViperXGripper[{command}]")
        self.command = command
        self.action_name = action_name
        self.client: Optional[ActionClient] = None
        self.node = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_viperx_gripper_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, PickArm, self.action_name)
        return True

    def initialise(self):
        if self.command not in ("open", "close"):
            self.feedback_message = f"Invalid gripper command: {self.command}"
            self.send_future = None
            return

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.feedback_message = f"Action server unavailable: {self.action_name}"
            self.send_future = None
            return

        goal_msg = PickArm.Goal()
        goal_msg.target_pose.header.frame_id = "vx300/base_link"
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.planning_group = "open_gripper" if self.command == "open" else "close_gripper"
        goal_msg.ee_link = ""
        goal_msg.pregrasp_offset_m = 0.0
        goal_msg.retreat_offset_m = 0.0
        goal_msg.gripper_close_position = -1.0
        goal_msg.use_cartesian_approach = False

        self.send_future = self.client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = f"Gripper command sent: {self.command}"

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE

        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.goal_handle is None and self.send_future.done():
            self.goal_handle = self.send_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.feedback_message = "Gripper goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.result_future is not None and self.result_future.done():
            wrapped = self.result_future.result()
            if wrapped is None:
                self.feedback_message = "Gripper result missing"
                return py_trees.common.Status.FAILURE
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"Gripper failed with status={wrapped.status}"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.client = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
