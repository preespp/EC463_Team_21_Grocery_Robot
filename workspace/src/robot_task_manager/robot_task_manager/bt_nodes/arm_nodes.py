import json
import math
import time
from typing import Optional

import py_trees
import rclpy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from robot_interfaces.action import PickArm
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

class RepositionArmToGoalPose(py_trees.behaviour.Behaviour):
    """
    Reads a target from bb.<goal_key> and commands the servo arm.

    Pose targets go to `/pick_arm_demo`, which computes joint targets and feeds
    them to the motor bridge. Command targets (e.g. `home`) go to the waypoint server.
    """
    def __init__(
        self,
        goal_key: str,
        bb=None,
        pose_action_name: str = "/pick_arm_demo",
        command_action_name: str = "/pick_arm_waypoint",
    ):
        super().__init__(f"RepositionArmToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.pose_action_name = pose_action_name
        self.command_action_name = command_action_name

        self.node = None
        self.pose_client = None
        self.command_client = None
        self.client = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_servo_arm_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.pose_client = ActionClient(self.node, PickArm, self.pose_action_name)
        self.command_client = ActionClient(self.node, PickArm, self.command_action_name)
        return True

    @staticmethod
    def _dict_to_pose_stamped(target_pose: dict) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = str(target_pose.get("frame_id", "base_link"))
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

    @staticmethod
    def _pose_to_dict(target_pose) -> dict:
        if isinstance(target_pose, PoseStamped):
            return {
                "frame_id": str(target_pose.header.frame_id or "base_link"),
                "x": float(target_pose.pose.position.x),
                "y": float(target_pose.pose.position.y),
                "z": float(target_pose.pose.position.z),
                "qx": float(target_pose.pose.orientation.x),
                "qy": float(target_pose.pose.orientation.y),
                "qz": float(target_pose.pose.orientation.z),
                "qw": float(target_pose.pose.orientation.w or 1.0),
            }
        return {
            "frame_id": str(target_pose.get("frame_id", "base_link")),
            "x": float(target_pose.get("x", target_pose.get("position", {}).get("x", 0.0))),
            "y": float(target_pose.get("y", target_pose.get("position", {}).get("y", 0.0))),
            "z": float(target_pose.get("z", target_pose.get("position", {}).get("z", 0.0))),
            "qx": float(target_pose.get("qx", target_pose.get("orientation", {}).get("x", 0.0))),
            "qy": float(target_pose.get("qy", target_pose.get("orientation", {}).get("y", 0.0))),
            "qz": float(target_pose.get("qz", target_pose.get("orientation", {}).get("z", 0.0))),
            "qw": float(target_pose.get("qw", target_pose.get("orientation", {}).get("w", 1.0))),
        }

    def initialise(self):
        target = getattr(self.bb, self.goal_key, None)
        if target is None:
            self.feedback_message = f"No target found for bb.{self.goal_key}"
            self.send_future = None
            return

        goal_msg = PickArm.Goal()
        goal_msg.target_pose.header.frame_id = "base_link"
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.ee_link = ""
        goal_msg.pregrasp_offset_m = 0.0
        goal_msg.retreat_offset_m = 0.0
        goal_msg.gripper_close_position = -1.0
        goal_msg.use_cartesian_approach = False

        if isinstance(target, dict) and isinstance(target.get("command"), str):
            self.client = self.command_client
            goal_msg.planning_group = str(target["command"])
        elif isinstance(target, PoseStamped) or isinstance(target, dict):
            self.client = self.pose_client
            goal_msg.planning_group = ""
            goal_msg.target_pose = (
                target if isinstance(target, PoseStamped) else self._dict_to_pose_stamped(target)
            )
            self.bb.arm_last_commanded_pose = self._pose_to_dict(target)
        else:
            self.feedback_message = f"Unsupported arm target type in bb.{self.goal_key}"
            self.send_future = None
            return

        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        action_name = (
            self.command_action_name
            if self.client is self.command_client
            else self.pose_action_name
        )
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.feedback_message = f"Action server unavailable: {action_name}"
            self.send_future = None
            return

        self.send_future = self.client.send_goal_async(goal_msg)
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = f"Sent arm goal from bb.{self.goal_key}"

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE

        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.goal_handle is None and self.send_future.done():
            self.goal_handle = self.send_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.feedback_message = "Servo arm goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if self.result_future is not None and self.result_future.done():
            wrapped = self.result_future.result()
            if wrapped is None:
                self.feedback_message = "Servo arm result missing"
                return py_trees.common.Status.FAILURE
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = f"Reached bb.{self.goal_key}"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"Servo arm failed with status={wrapped.status}"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        self.client = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None


class VerifyPosition(py_trees.behaviour.Behaviour):
    """
    Detect the current item with robot_vision and decide whether another
    correction move is required before gripping.
    """
    def __init__(
        self,
        bb=None,
        detections_topic: str = "/detections_json",
        target_frame: str = "base_link",
        min_confidence: float = 0.60,
        min_depth_m: float = 0.08,
        max_depth_m: float = 0.90,
        position_tolerance_m: float = 0.05,
        success_on_detect: bool = False,
    ):
        super().__init__("VerifyPosition")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.detections_topic = detections_topic
        self.target_frame = target_frame
        self.min_confidence = min_confidence
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.position_tolerance_m = position_tolerance_m
        self.success_on_detect = success_on_detect

        self.node = None
        self.sub = None
        self.last_detections_json = None
        self.last_detection_time = None
        self.detection_timeout_sec = 2.0
        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_error_time = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_verify_servo_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.sub = self.node.create_subscription(
            String,
            self.detections_topic,
            self._on_detections,
            10,
        )
        return True

    def _on_detections(self, msg: String):
        self.last_detections_json = msg.data
        self.last_detection_time = time.time()

    def _get_expected_product_names(self) -> list[str]:
        current_item = getattr(self.bb, "current_item", None)
        if current_item is None:
            return []
        item_name = str(getattr(current_item, "name", "")).strip().lower()
        return [item_name] if item_name else []

    @staticmethod
    def _fuzzy_match_product(detected_name: str, expected_names: list[str]) -> bool:
        detected = detected_name.lower().strip()
        for expected in expected_names:
            expected_clean = expected.lower().strip()
            if detected == expected_clean:
                return True
            detected_words = set(detected.replace("_", " ").split())
            expected_words = set(expected_clean.replace("_", " ").split())
            if expected_words and detected_words:
                if detected_words.issubset(expected_words) or expected_words.issubset(detected_words):
                    return True
                if len(detected_words & expected_words) >= len(expected_words) * 0.8:
                    return True
        return False

    def _filter_detections(self, detections: list) -> Optional[dict]:
        expected_names = self._get_expected_product_names()
        if not expected_names:
            self.feedback_message = "No expected product in bb.current_item"
            return None

        best_detection = None
        best_confidence = 0.0
        for det in detections:
            if not isinstance(det, dict):
                continue
            class_name = str(det.get("class_name", "")).lower()
            if not self._fuzzy_match_product(class_name, expected_names):
                continue
            confidence = float(det.get("confidence", 0.0))
            if confidence < self.min_confidence:
                continue
            distance_m = float(det.get("distance_m", 0.0))
            if distance_m <= 0 or distance_m < self.min_depth_m or distance_m > self.max_depth_m:
                continue
            if confidence > best_confidence:
                best_confidence = confidence
                best_detection = det
        if best_detection is None:
            self.feedback_message = "No matching detection passed filtering"
        return best_detection

    def _transform_point_to_target(
        self,
        point_m: list[float],
        source_frame: str,
    ) -> Optional[tuple[float, float, float]]:
        point = PointStamped()
        point.header.stamp.sec = 0
        point.header.stamp.nanosec = 0
        point.header.frame_id = source_frame
        point.point.x = float(point_m[0])
        point.point.y = float(point_m[1])
        point.point.z = float(point_m[2])

        try:
            transformed = self.tf_buffer.transform(
                point,
                self.target_frame,
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            now = time.time()
            if self.last_tf_error_time is None or now - self.last_tf_error_time > 2.0:
                self.last_tf_error_time = now
                self.feedback_message = (
                    f"TF transform failed {source_frame} -> {self.target_frame}: {exc}"
                )
            return None

        return (
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        )

    @staticmethod
    def _distance_m(a: dict, b: dict) -> float:
        return math.sqrt(
            (float(a.get("x", 0.0)) - float(b.get("x", 0.0))) ** 2
            + (float(a.get("y", 0.0)) - float(b.get("y", 0.0))) ** 2
            + (float(a.get("z", 0.0)) - float(b.get("z", 0.0))) ** 2
        )

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.last_detections_json is None:
            self.feedback_message = "Waiting for camera detections..."
            return py_trees.common.Status.RUNNING

        if time.time() - self.last_detection_time > self.detection_timeout_sec:
            self.feedback_message = "Detection stream stale (timeout)"
            return py_trees.common.Status.FAILURE

        try:
            payload = json.loads(self.last_detections_json)
            detections = payload.get("detections", [])
            camera_frame = str(payload.get("camera_optical_frame", "")).strip()
        except (json.JSONDecodeError, TypeError):
            self.feedback_message = "Failed to parse detection JSON"
            return py_trees.common.Status.FAILURE

        if not detections:
            self.feedback_message = "Camera sees no objects"
            return py_trees.common.Status.FAILURE
        if not camera_frame:
            self.feedback_message = "detections_json missing camera_optical_frame"
            return py_trees.common.Status.FAILURE

        best_detection = self._filter_detections(detections)
        if best_detection is None:
            return py_trees.common.Status.FAILURE

        raw_point = best_detection.get("point_camera_optical_m", [])
        if not isinstance(raw_point, list) or len(raw_point) != 3:
            self.feedback_message = "Detection missing point_camera_optical_m"
            return py_trees.common.Status.FAILURE

        target_xyz = self._transform_point_to_target(raw_point, camera_frame)
        if target_xyz is None:
            return py_trees.common.Status.FAILURE

        detected_pose = {
            "frame_id": self.target_frame,
            "x": float(target_xyz[0]),
            "y": float(target_xyz[1]),
            "z": float(target_xyz[2]),
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "distance_m": float(best_detection["distance_m"]),
            "source_frame": camera_frame,
            "class_name": str(best_detection["class_name"]),
            "confidence": float(best_detection["confidence"]),
        }
        self.bb.detected_object_pose = detected_pose
        self.bb.pose = {
            key: detected_pose[key]
            for key in ("frame_id", "x", "y", "z", "qx", "qy", "qz", "qw")
        }

        if self.success_on_detect:
            self.feedback_message = (
                f"Detection ready for approach: {detected_pose['class_name']} "
                f"conf={detected_pose['confidence']:.2f}"
            )
            return py_trees.common.Status.SUCCESS

        last_commanded_pose = getattr(self.bb, "arm_last_commanded_pose", None)
        if last_commanded_pose is None:
            self.feedback_message = "No previous arm target to verify against; using detected pose"
            return py_trees.common.Status.FAILURE

        error_m = self._distance_m(detected_pose, last_commanded_pose)
        if error_m <= self.position_tolerance_m:
            self.feedback_message = f"Alignment verified (error={error_m:.3f}m)"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Alignment needs correction (error={error_m:.3f}m)"
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        del new_status
        self.last_detections_json = None

class MoveGripper(py_trees.behaviour.Behaviour):
    """
    command: "open" or "close"
    """
    def __init__(self, command: str, action_name: str = "/pick_arm_waypoint"):
        super().__init__(f"MoveGripper[{command}]")
        self.command = command
        self.action_name = action_name
        self.node = None
        self.client = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_servo_gripper_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, PickArm, self.action_name)
        return True

    def initialise(self):
        if self.command not in ("open", "close"):
            self.send_future = None
            self.feedback_message = f"Invalid gripper command: {self.command}"
            return

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.send_future = None
            self.feedback_message = f"Action server unavailable: {self.action_name}"
            return

        goal_msg = PickArm.Goal()
        goal_msg.target_pose.header.frame_id = "base_link"
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
                self.feedback_message = f"Gripper {self.command} complete"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"Gripper failed with status={wrapped.status}"
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
