import json
import time
from copy import deepcopy
from typing import Optional

import py_trees
import rclpy
import tf2_geometry_msgs  # noqa: F401 - required to register PointStamped transforms
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.duration import Duration
from robot_interfaces.action import PickArm
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class RepositionViperXArm(py_trees.behaviour.Behaviour):
    """
    Read a target from blackboard and send a PickArm action goal to ViperX.

    The target may be:
      - dict with key command for a configured preset joint state
      - geometry_msgs.msg.PoseStamped
      - dict with keys frame_id, x, y, z, qx, qy, qz, qw
      - dict with keys frame_id, position{x,y,z}, orientation{x,y,z,w}

    BT convention:
      - preset arm states should be stored as {"command": "<preset_name>"}
      - live Cartesian arm targets should be stored as pose dictionaries
    """

    def __init__(self, goal_key: str, bb=None, action_name: str = "/pick_viperx"):
        super().__init__(f"RepositionViperXArm[{goal_key}]")
        self.goal_key = goal_key
        self.action_name = action_name
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()

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

    def _make_command_goal(self, command: str) -> PickArm.Goal:
        goal_msg = PickArm.Goal()
        goal_msg.target_pose.header.frame_id = "vx300/base_link"
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.target_pose.pose.orientation.w = 1.0
        goal_msg.planning_group = command
        goal_msg.ee_link = ""
        goal_msg.pregrasp_offset_m = 0.0
        goal_msg.retreat_offset_m = 0.0
        goal_msg.gripper_close_position = -1.0
        goal_msg.use_cartesian_approach = False
        return goal_msg

    def _make_pose_goal(self, target_pose) -> PickArm.Goal:
        goal_msg = PickArm.Goal()
        goal_msg.planning_group = "arm"
        goal_msg.ee_link = ""
        goal_msg.pregrasp_offset_m = 0.0
        goal_msg.retreat_offset_m = 0.0
        goal_msg.gripper_close_position = -1.0
        goal_msg.use_cartesian_approach = False

        if isinstance(target_pose, PoseStamped):
            goal_msg.target_pose = target_pose
        else:
            goal_msg.target_pose = self._dict_to_pose_stamped(target_pose)
        return goal_msg

    @staticmethod
    def _is_command_target(target) -> bool:
        return isinstance(target, dict) and isinstance(target.get("command"), str)

    @staticmethod
    def _is_pose_dict_target(target) -> bool:
        return isinstance(target, dict) and "command" not in target

    def initialise(self):
        target = getattr(self.bb, self.goal_key, None)
        if target is None:
            self.feedback_message = f"No pose found for bb.{self.goal_key}"
            self.send_future = None
            return

        if not self.client.wait_for_server(timeout_sec=2.0):
            self.feedback_message = f"Action server unavailable: {self.action_name}"
            self.send_future = None
            return

        if self._is_command_target(target):
            goal_msg = self._make_command_goal(target["command"])
        elif isinstance(target, PoseStamped) or self._is_pose_dict_target(target):
            goal_msg = self._make_pose_goal(target)
        else:
            self.feedback_message = (
                f"Unsupported target in bb.{self.goal_key}; expected "
                "{'command': str}, PoseStamped, or pose dict"
            )
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
    Detect object via camera subscription and verify it matches expected product.
    
    Uses blackboard to:
    1. Get expected product name (bb.current_item.name)
    2. Filter detections by product match, confidence, depth
    3. Store best detection in bb.detected_object_pose
    
    Communication: Subscribes to /detections_json (published by robot_vision)
    """

    def __init__(
        self,
        bb=None,
        detections_topic: str = "/detections_json",
        min_confidence: float = 0.60,
        min_depth_m: float = 0.08,
        max_depth_m: float = 0.70,
        target_frame: str = "vx300/base_link",
    ):
        super().__init__("VerifyViperXPosition")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.detections_topic = detections_topic
        self.min_confidence = min_confidence
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.target_frame = target_frame

        self.node = None
        self.sub = None
        self.last_detections_json = None
        self.detection_timeout_sec = 2.0
        self.last_detection_time = None
        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_error_time = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_verify_viperx_{id(self) & 0xFFFF:x}"
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

    def _on_detections(self, msg: String):
        """Callback when camera publishes detections."""
        self.last_detections_json = msg.data
        self.last_detection_time = time.time()

    def _get_expected_product_names(self) -> list:
        """Get expected product name from blackboard."""
        current_item = getattr(self.bb, "current_item", None)
        if current_item is None:
            return []
        
        item_name = str(getattr(current_item, "name", "")).lower().strip()
        return [item_name] if item_name else []

    def _fuzzy_match_product(self, detected_name: str, expected_names: list) -> bool:
        """
        Check if detected product matches expected product.
        Supports variations like "water_bottle" vs "water bottle".
        """
        detected = detected_name.lower().strip()
        
        for expected in expected_names:
            expected_clean = expected.lower().strip()
            
            # Exact match
            if detected == expected_clean:
                return True
            
            # Fuzzy: normalize underscores/spaces and check word overlap
            detected_words = set(detected.replace("_", " ").split())
            expected_words = set(expected_clean.replace("_", " ").split())
            
            if expected_words and detected_words:
                overlap = detected_words & expected_words
                # 80% word overlap = match
                if len(overlap) >= len(expected_words) * 0.8:
                    return True
        
        return False

    def _filter_detections(self, detections: list) -> dict:
        """
        Filter detections by:
        1. Product name match (using bb.current_item)
        2. Confidence threshold
        3. Depth range
        Returns best detection or None.
        """
        expected_names = self._get_expected_product_names()
        if not expected_names:
            self.feedback_message = "No expected product in bb.current_item"
            return None

        best_detection = None
        best_confidence = 0.0
        rejection_stats = {
            "product_mismatch": 0,
            "low_confidence": 0,
            "bad_depth": 0,
        }

        for det in detections:
            if not isinstance(det, dict):
                continue

            # 1. Product name match
            class_name = str(det.get("class_name", "")).lower()
            if not self._fuzzy_match_product(class_name, expected_names):
                rejection_stats["product_mismatch"] += 1
                continue

            # 2. Confidence check
            confidence = float(det.get("confidence", 0.0))
            if confidence < self.min_confidence:
                rejection_stats["low_confidence"] += 1
                continue

            # 3. Depth range check
            distance_m = float(det.get("distance_m", 0.0))
            if distance_m <= 0 or distance_m < self.min_depth_m or distance_m > self.max_depth_m:
                rejection_stats["bad_depth"] += 1
                continue

            # Select best (highest confidence)
            if confidence > best_confidence:
                best_confidence = confidence
                best_detection = det

        if best_detection is None:
            reasons = ", ".join(f"{k}={v}" for k, v in rejection_stats.items() if v > 0)
            self.feedback_message = f"Detection filtered out: {reasons or 'none passed'}"
            return None

        return best_detection

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.last_detections_json is None:
            self.feedback_message = "Waiting for camera detections..."
            return py_trees.common.Status.RUNNING

        # Check if detections are stale
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

        # Filter and select best detection
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

        # Store in blackboard for next node
        self.bb.detected_object_pose = {
            "x": float(target_xyz[0]),
            "y": float(target_xyz[1]),
            "z": float(target_xyz[2]),
            "distance_m": float(best_detection["distance_m"]),
            "frame_id": self.target_frame,
            "source_frame": camera_frame,
            "class_name": str(best_detection["class_name"]),
            "confidence": float(best_detection["confidence"]),
        }

        self.feedback_message = (
            f"Detected '{best_detection['class_name']}' "
            f"distance={best_detection['distance_m']:.3f}m "
            f"confidence={best_detection['confidence']:.2f}"
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        del new_status
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.sub = None
        self.last_detections_json = None
        self.tf_buffer = None
        self.tf_listener = None


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


class SelectBasketSlot(py_trees.behaviour.Behaviour):
    """
    Selects the appropriate basket slot based on item type and bottle count.
    
    For bottles: Uses slots 0, 1, 2 (one per bottle) - increments bb.basket_bottle_count
    For non-bottles: Uses slot 3 (random items slot)
    
    Sets bb.basket_pose to the selected basket pose from bb.basket_poses.
    """

    def __init__(self, bb=None):
        super().__init__("SelectBasketSlot")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()

    def update(self):
        current_item = getattr(self.bb, "current_item", None)
        if current_item is None:
            self.feedback_message = "No current item set"
            return py_trees.common.Status.FAILURE

        # Get item type/name to determine if it's a bottle
        item_name = str(getattr(current_item, "name", "")).lower()
        is_bottle = "bottle" in item_name or "water" in item_name

        basket_poses = getattr(self.bb, "basket_poses", [None, None, None, None])
        bottle_count = getattr(self.bb, "basket_bottle_count", 0)

        if is_bottle:
            # Use bottle slots 0, 1, 2
            if bottle_count >= 3:
                self.feedback_message = "All bottle slots full"
                return py_trees.common.Status.FAILURE
            
            selected_slot = bottle_count
            self.bb.basket_pose = basket_poses[selected_slot]
            self.bb.basket_bottle_count = bottle_count + 1
            self.feedback_message = f"Selected bottle slot {selected_slot + 1}/3"
        else:
            # Use random items slot (slot 3)
            selected_slot = 3
            self.bb.basket_pose = basket_poses[selected_slot]
            self.feedback_message = "Selected random items slot"

        if self.bb.basket_pose is None:
            self.feedback_message = (
                f"Basket pose not initialized at slot {selected_slot}"
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS


class MoveToDetectedPose(py_trees.behaviour.Behaviour):
    """
    Moves arm to the detected object pose (stored in bb.detected_object_pose by VerifyViperXPosition).
    Uses the RepositionViperXArm action with the detected pose.
    """

    def __init__(self, bb=None, action_name: str = "/pick_viperx"):
        super().__init__("MoveToDetectedPose")
        self.action_name = action_name
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()

        self.node = None
        self.client: Optional[ActionClient] = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_viperx_move_detected_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, PickArm, self.action_name)
        return True

    def initialise(self):
        target_pose = getattr(self.bb, "detected_object_pose", None)
        if target_pose is None:
            self.feedback_message = "No detected pose available (bb.detected_object_pose is None)"
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

        # Convert detected pose to PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = target_pose.get("frame_id", "vx300/base_link")
        pose_msg.pose.position.x = float(target_pose.get("x", 0.0))
        pose_msg.pose.position.y = float(target_pose.get("y", 0.0))
        pose_msg.pose.position.z = float(target_pose.get("z", 0.0))
        pose_msg.pose.orientation.w = 1.0

        goal_msg.target_pose = pose_msg
        goal_msg.target_pose.header.stamp = self.node.get_clock().now().to_msg()
        self.send_future = self.client.send_goal_async(goal_msg)
        self.feedback_message = "Moving to detected object pose"

    def update(self):
        if self.send_future is None:
            return py_trees.common.Status.FAILURE

        rclpy.spin_once(self.node, timeout_sec=0.05)

        if self.goal_handle is None and self.send_future.done():
            self.goal_handle = self.send_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.feedback_message = "Move goal rejected"
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            self.feedback_message = "Move goal accepted, executing"
            return py_trees.common.Status.RUNNING

        if self.result_future is not None and self.result_future.done():
            wrapped = self.result_future.result()
            if wrapped is None:
                self.feedback_message = "Move result missing"
                return py_trees.common.Status.FAILURE
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = "Successfully moved to detected object pose"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"Move failed with status={wrapped.status}"
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
