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


def _resolve_base_frame(bb, fallback: str = "vx300s/base_link") -> str:
    frame = str(getattr(bb, "arm_base_frame", fallback)).strip()
    return frame or fallback


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
        if self.client is not None:
            return True
        if not rclpy.ok():
            return False
        node_name = f"bt_viperx_arm_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, PickArm, self.action_name)
        return True

    @staticmethod
    def _dict_to_pose_stamped(target_pose: dict, default_frame: str) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = str(target_pose.get("frame_id", default_frame))
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
        goal_msg.target_pose.header.frame_id = _resolve_base_frame(self.bb)
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
            goal_msg.target_pose = self._dict_to_pose_stamped(
                target_pose,
                default_frame=_resolve_base_frame(self.bb),
            )
            goal_msg.ee_link = str(target_pose.get("ee_link", "")).strip()
            goal_msg.pregrasp_offset_m = float(target_pose.get("pregrasp_offset_m", 0.0))
            goal_msg.retreat_offset_m = float(target_pose.get("retreat_offset_m", 0.0))
            goal_msg.gripper_close_position = float(
                target_pose.get("gripper_close_position", -1.0)
            )
            goal_msg.use_cartesian_approach = bool(
                target_pose.get("use_cartesian_approach", False)
            )
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
            result_msg = ""
            if getattr(wrapped, "result", None) is not None:
                result_msg = str(getattr(wrapped.result, "message", "") or "")
            self.feedback_message = (
                f"ViperX failed with status={wrapped.status}"
                + (f": {result_msg}" if result_msg else "")
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def shutdown(self):
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
        min_confidence: float = 0.50,
        min_depth_m: float = 0.08,
        max_depth_m: float = 0.70,
        target_frame: Optional[str] = None,
        target_stability_count: int = 4,
        target_match_distance_m: float = 0.03,
        diagnostics_log_period_sec: float = 2.0,
        search_timeout_sec: Optional[float] = None,
    ):
        super().__init__("VerifyViperXPosition")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.detections_topic = detections_topic
        self.min_confidence = min_confidence
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.target_frame = str(target_frame).strip() if target_frame is not None else None
        self.target_stability_count = int(target_stability_count)
        self.target_match_distance_m = float(target_match_distance_m)
        self.diagnostics_log_period_sec = float(diagnostics_log_period_sec)
        self.search_timeout_sec = (
            float(search_timeout_sec) if search_timeout_sec is not None else None
        )

        self.node = None
        self.sub = None
        self.last_detections_json = None
        self.detection_timeout_sec = 2.0
        self.last_detection_time = None
        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_error_time = None
        self.last_diagnostics_log_time = None
        self.last_stable_xyz = None
        self.stable_xyz_samples = []
        self.stable_count = 0
        self.search_start_time = None

    def initialise(self):
        self.search_start_time = time.monotonic()
        self.last_stable_xyz = None
        self.stable_xyz_samples = []
        self.stable_count = 0
        self.bb.detected_object_pose = None

    def setup(self, **kwargs):
        if getattr(self.bb, "shared_tf_buffer", None) is not None:
            return True
        if self.sub is not None:
            return True
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

    def _target_frame(self) -> str:
        return self.target_frame or _resolve_base_frame(self.bb)

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

        tf_buffer = getattr(self.bb, "shared_tf_buffer", None) or self.tf_buffer
        if tf_buffer is None:
            self.feedback_message = "TF buffer unavailable"
            return None

        try:
            transformed = tf_buffer.transform(
                point,
                self._target_frame(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            now = time.time()
            if self.last_tf_error_time is None or now - self.last_tf_error_time > 2.0:
                self.last_tf_error_time = now
                self.feedback_message = (
                    f"TF transform failed {source_frame} -> {self._target_frame()}: {exc}"
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

    @staticmethod
    def _parse_target_classes(text: str) -> set[str]:
        return {
            part.strip().lower()
            for part in str(text).split(",")
            if part.strip()
        }

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        dx = float(a[0]) - float(b[0])
        dy = float(a[1]) - float(b[1])
        dz = float(a[2]) - float(b[2])
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    @staticmethod
    def _average_xyz(samples: list[tuple[float, float, float]]) -> tuple[float, float, float]:
        if not samples:
            return 0.0, 0.0, 0.0
        count = float(len(samples))
        return (
            sum(sample[0] for sample in samples) / count,
            sum(sample[1] for sample in samples) / count,
            sum(sample[2] for sample in samples) / count,
        )

    def _should_log_diagnostics(self) -> bool:
        if self.diagnostics_log_period_sec <= 0.0:
            return False
        now = time.time()
        if self.last_diagnostics_log_time is None:
            self.last_diagnostics_log_time = now
            return True
        if now - self.last_diagnostics_log_time >= self.diagnostics_log_period_sec:
            self.last_diagnostics_log_time = now
            return True
        return False

    def _maybe_log_diagnostics(self, message: str) -> None:
        if self.node is None or not self._should_log_diagnostics():
            return
        self.node.get_logger().info(message)

    def _timeout_status(self, reason: str):
        if self.search_timeout_sec is None or self.search_start_time is None:
            return None

        elapsed = time.monotonic() - self.search_start_time
        if elapsed < self.search_timeout_sec:
            return None

        self.last_stable_xyz = None
        self.stable_xyz_samples = []
        self.stable_count = 0
        self.feedback_message = (
            f"{reason}; scan timed out after {elapsed:.1f}s"
        )
        return py_trees.common.Status.FAILURE

    def _effective_target_classes(self) -> set[str]:
        target_classes = getattr(self.bb, "viperx_target_classes", None)
        if isinstance(target_classes, (list, tuple, set)):
            parsed = {
                str(value).strip().lower()
                for value in target_classes
                if str(value).strip()
            }
            if parsed:
                return parsed

        target_classes_text = str(
            getattr(self.bb, "viperx_target_classes_text", "")
        ).strip()
        parsed = self._parse_target_classes(target_classes_text)
        if parsed:
            return parsed

        return set()

    def _get_expected_product_names(self) -> list:
        """Get expected product name from blackboard."""
        current_item = getattr(self.bb, "current_item", None)
        if current_item is None:
            return []
        
        item_name = str(getattr(current_item, "name", "")).lower().strip()
        return [item_name] if item_name else []

    def _effective_min_confidence(self) -> float:
        return float(
            getattr(self.bb, "viperx_detection_min_confidence", self.min_confidence)
        )

    def _effective_target_stability_count(self) -> int:
        return int(
            getattr(self.bb, "viperx_target_stability_count", self.target_stability_count)
        )

    def _effective_target_match_distance_m(self) -> float:
        return float(
            getattr(self.bb, "viperx_target_match_distance_m", self.target_match_distance_m)
        )

    def _workspace_limit(self, attr_name: str, fallback: float) -> float:
        return float(getattr(self.bb, attr_name, fallback))

    def _within_workspace(self, xyz: tuple[float, float, float]) -> bool:
        x, y, z = xyz
        return (
            self._workspace_limit("viperx_workspace_min_x", -0.50)
            <= float(x)
            <= self._workspace_limit("viperx_workspace_max_x", 0.80)
            and self._workspace_limit("viperx_workspace_min_y", -0.80)
            <= float(y)
            <= self._workspace_limit("viperx_workspace_max_y", 0.80)
            and self._workspace_limit("viperx_workspace_min_z", -0.20)
            <= float(z)
            <= self._workspace_limit("viperx_workspace_max_z", 0.80)
        )

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

    def _select_best_detection(
        self,
        detections: list,
        camera_frame: str,
    ) -> tuple[Optional[dict], dict]:
        target_classes = self._effective_target_classes()
        expected_names = self._get_expected_product_names()

        candidates = []
        summary = {
            "total": len(detections),
            "class_mismatch": 0,
            "low_confidence": 0,
            "bad_depth": 0,
            "bad_point": 0,
            "tf_fail": 0,
            "out_of_workspace": 0,
            "example_workspace_xyz": None,
        }

        for det in detections:
            if not isinstance(det, dict):
                summary["bad_point"] = int(summary["bad_point"]) + 1
                continue

            cls_name = str(det.get("class_name", "")).strip().lower()
            if target_classes:
                if cls_name not in target_classes:
                    summary["class_mismatch"] = int(summary["class_mismatch"]) + 1
                    continue
            elif expected_names and not self._fuzzy_match_product(cls_name, expected_names):
                summary["class_mismatch"] = int(summary["class_mismatch"]) + 1
                continue

            confidence = float(det.get("confidence", 0.0))
            if confidence < self._effective_min_confidence():
                summary["low_confidence"] = int(summary["low_confidence"]) + 1
                continue

            depth_m = float(det.get("distance_m", 0.0))
            if depth_m <= 0.0 or depth_m < self.min_depth_m or depth_m > self.max_depth_m:
                summary["bad_depth"] = int(summary["bad_depth"]) + 1
                continue

            point_m = det.get("point_camera_optical_m", [])
            if not isinstance(point_m, list) or len(point_m) != 3:
                summary["bad_point"] = int(summary["bad_point"]) + 1
                continue

            transformed = self._transform_point_to_target(point_m, camera_frame)
            if transformed is None:
                summary["tf_fail"] = int(summary["tf_fail"]) + 1
                continue

            if not self._within_workspace(transformed):
                summary["out_of_workspace"] = int(summary["out_of_workspace"]) + 1
                if summary["example_workspace_xyz"] is None:
                    summary["example_workspace_xyz"] = transformed
                continue

            candidates.append({
                "xyz": transformed,
                "class_name": cls_name or "unknown",
                "confidence": confidence,
                "depth_m": depth_m,
                "surface_distance_m": float(det.get("surface_distance_m", depth_m)),
                "bbox": list(det.get("bbox", [])) if isinstance(det.get("bbox"), list) else [],
                "center_px": list(det.get("center_px", [])) if isinstance(det.get("center_px"), list) else [],
                "grasp_px": list(det.get("grasp_px", [])) if isinstance(det.get("grasp_px"), list) else [],
                "grasp_strategy": str(det.get("grasp_strategy", "bbox_center")),
                "partial_visibility": bool(det.get("partial_visibility", False)),
                "visibility_case": str(det.get("visibility_case", "full_or_unknown")),
                "visible_ratio_estimate": float(det.get("visible_ratio_estimate", 1.0)),
            })

        if not candidates:
            return None, summary

        candidates.sort(key=lambda candidate: (float(candidate["depth_m"]), -float(candidate["confidence"])))
        return candidates[0], summary

    def update(self):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        detections_json = getattr(self.bb, "latest_detections_json", None)
        detection_time = getattr(self.bb, "latest_detection_time", None)
        if detections_json is None:
            detections_json = self.last_detections_json

        if detections_json is None:
            timeout_status = self._timeout_status("Waiting for camera detections")
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = "Waiting for camera detections..."
            return py_trees.common.Status.RUNNING

        # Check if detections are stale
        if detection_time is not None:
            stale_sec = (self.node.get_clock().now() - detection_time).nanoseconds / 1e9 if self.node is not None else 0.0
        else:
            stale_sec = time.time() - self.last_detection_time if self.last_detection_time is not None else 1e9

        if stale_sec > self.detection_timeout_sec:
            timeout_status = self._timeout_status("Waiting for fresh camera detections")
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = "Waiting for fresh camera detections..."
            return py_trees.common.Status.RUNNING

        try:
            payload = json.loads(detections_json)
            detections = payload.get("detections", [])
            camera_frame = str(payload.get("camera_optical_frame", "")).strip()
        except (json.JSONDecodeError, TypeError):
            timeout_status = self._timeout_status("Waiting for valid detection JSON")
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = "Waiting for valid detection JSON..."
            return py_trees.common.Status.RUNNING

        if not detections:
            self.stable_count = 0
            self.last_stable_xyz = None
            self.stable_xyz_samples = []
            timeout_status = self._timeout_status("Camera sees no objects")
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = "Camera sees no objects"
            self._maybe_log_diagnostics(
                f"Auto-pick sees camera stream from {camera_frame or '<unknown>'}, but detections list is empty."
            )
            return py_trees.common.Status.RUNNING

        if not camera_frame:
            timeout_status = self._timeout_status(
                "Waiting for detections_json with camera_optical_frame"
            )
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = "Waiting for detections_json with camera_optical_frame..."
            return py_trees.common.Status.RUNNING

        best_detection, summary = self._select_best_detection(detections, camera_frame)
        if best_detection is None:
            self.stable_count = 0
            self.last_stable_xyz = None
            self.stable_xyz_samples = []
            message = (
                "Auto-pick rejected current detections: "
                f"total={int(summary['total'])} "
                f"class_mismatch={int(summary['class_mismatch'])} "
                f"low_conf={int(summary['low_confidence'])} "
                f"bad_depth={int(summary['bad_depth'])} "
                f"bad_point={int(summary['bad_point'])} "
                f"tf_fail={int(summary['tf_fail'])} "
                f"out_of_workspace={int(summary['out_of_workspace'])}"
            )
            example_xyz = summary.get("example_workspace_xyz")
            if isinstance(example_xyz, tuple):
                message += (
                    " example_base_xyz="
                    f"({example_xyz[0]:.3f}, {example_xyz[1]:.3f}, {example_xyz[2]:.3f})"
                )
            timeout_status = self._timeout_status(message)
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = message
            self._maybe_log_diagnostics(message)
            return py_trees.common.Status.RUNNING

        target_xyz = best_detection["xyz"]
        if (
            self.last_stable_xyz is not None
            and self._distance(self.last_stable_xyz, target_xyz)
            <= self._effective_target_match_distance_m()
        ):
            self.stable_count += 1
            self.last_stable_xyz = target_xyz
            self.stable_xyz_samples.append(target_xyz)
        else:
            self.stable_count = 1
            self.last_stable_xyz = target_xyz
            self.stable_xyz_samples = [target_xyz]

        if self.stable_count < self._effective_target_stability_count():
            averaged_xyz = self._average_xyz(self.stable_xyz_samples)
            waiting_message = (
                "Candidate accepted but waiting for stability: "
                f"{self.stable_count}/{self._effective_target_stability_count()} "
                f"class={best_detection['class_name']} "
                f"conf={float(best_detection['confidence']):.2f} "
                f"base_xyz=({averaged_xyz[0]:.3f}, {averaged_xyz[1]:.3f}, {averaged_xyz[2]:.3f})"
            )
            timeout_status = self._timeout_status(waiting_message)
            if timeout_status is not None:
                return timeout_status
            self.feedback_message = (
                waiting_message
            )
            self._maybe_log_diagnostics(self.feedback_message)
            return py_trees.common.Status.RUNNING

        averaged_xyz = self._average_xyz(self.stable_xyz_samples)
        self.bb.detected_object_pose = {
            "x": float(averaged_xyz[0]),
            "y": float(averaged_xyz[1]),
            "z": float(averaged_xyz[2]),
            "distance_m": float(best_detection["depth_m"]),
            "surface_distance_m": float(best_detection.get("surface_distance_m", best_detection["depth_m"])),
            "frame_id": self._target_frame(),
            "source_frame": camera_frame,
            "class_name": str(best_detection["class_name"]),
            "confidence": float(best_detection["confidence"]),
            "bbox": list(best_detection.get("bbox", [])),
            "center_px": list(best_detection.get("center_px", [])),
            "grasp_px": list(best_detection.get("grasp_px", [])),
            "grasp_strategy": str(best_detection.get("grasp_strategy", "bbox_center")),
            "partial_visibility": bool(best_detection.get("partial_visibility", False)),
            "visibility_case": str(best_detection.get("visibility_case", "full_or_unknown")),
            "visible_ratio_estimate": float(best_detection.get("visible_ratio_estimate", 1.0)),
        }
        self.stable_count = 0
        self.last_stable_xyz = None
        self.stable_xyz_samples = []

        self.feedback_message = (
            f"Detected '{best_detection['class_name']}' "
            f"distance={best_detection['depth_m']:.3f}m "
            f"confidence={best_detection['confidence']:.2f} "
            f"strategy={str(best_detection.get('grasp_strategy', 'bbox_center'))}"
        )
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        del new_status
        self.last_detections_json = None
        self.last_detection_time = None
        self.last_tf_error_time = None
        self.last_diagnostics_log_time = None
        self.last_stable_xyz = None
        self.stable_xyz_samples = []
        self.stable_count = 0
        self.search_start_time = None

    def shutdown(self):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.sub = None
        self.last_detections_json = None
        self.last_detection_time = None
        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_error_time = None
        self.last_diagnostics_log_time = None
        self.last_stable_xyz = None
        self.stable_xyz_samples = []
        self.stable_count = 0
        self.search_start_time = None


class LockCurrentViperXOrientation(py_trees.behaviour.Behaviour):
    """
    Lock the current EE orientation into bb.locked_pick_orientation_xyzw.

    This is useful before scanning so later grasp/pregrasp/lift poses reuse a stable
    orientation instead of inheriting whatever yaw the search motion ended on.
    """

    def __init__(
        self,
        bb=None,
        orientation_frame: Optional[str] = None,
        timeout_sec: float = 2.0,
    ):
        super().__init__("LockCurrentViperXOrientation")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.orientation_frame = (
            str(orientation_frame).strip() if orientation_frame is not None else None
        )
        self.timeout_sec = float(timeout_sec)
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None
        self.start_time = None

    def setup(self, **kwargs):
        if getattr(self.bb, "shared_tf_buffer", None) is not None:
            return True
        if self.tf_listener is not None:
            return True
        if not rclpy.ok():
            return False
        node_name = f"bt_lock_viperx_orientation_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        return True

    def initialise(self):
        self.start_time = time.monotonic()

    def _resolve_orientation_frame(self) -> str:
        if self.orientation_frame:
            return self.orientation_frame
        frame = str(getattr(self.bb, "viperx_ee_orientation_frame", "")).strip()
        return frame or "vx300s/ee_gripper_link"

    def update(self):
        locked = getattr(self.bb, "locked_pick_orientation_xyzw", None)
        if isinstance(locked, (list, tuple)) and len(locked) == 4:
            self.feedback_message = "Pick orientation already locked"
            return py_trees.common.Status.SUCCESS

        tf_buffer = getattr(self.bb, "shared_tf_buffer", None) or self.tf_buffer
        if tf_buffer is None:
            self.feedback_message = "TF buffer unavailable for orientation lock"
            return py_trees.common.Status.FAILURE

        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        try:
            transform = tf_buffer.lookup_transform(
                _resolve_base_frame(self.bb),
                self._resolve_orientation_frame(),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            elapsed = 0.0 if self.start_time is None else time.monotonic() - self.start_time
            self.feedback_message = f"Waiting to lock pick orientation: {exc}"
            if elapsed >= self.timeout_sec:
                self.feedback_message = (
                    "Unable to lock pick orientation before scan: "
                    f"{exc}"
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        q = transform.transform.rotation
        self.bb.locked_pick_orientation_xyzw = (
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        self.feedback_message = "Locked pick orientation from current EE pose"
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        del new_status
        self.start_time = None

    def shutdown(self):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None
        self.start_time = None


class PrepareDetectedPickPoses(py_trees.behaviour.Behaviour):
    """
    Build the safer old auto-pick approach poses from a live detected pose.

    Generates:
      - bb.pregrasp_pose
      - bb.grasp_pose
      - bb.lift_pose
    """

    def __init__(
        self,
        bb=None,
        pregrasp_offset_z_m: float = 0.15,
        grasp_offset_z_m: float = 0.0,
        lift_offset_z_m: float = 0.20,
        workspace_min_x: float = -0.50,
        workspace_max_x: float = 0.80,
        workspace_min_y: float = -0.80,
        workspace_max_y: float = 0.80,
        workspace_min_z: float = -0.20,
        workspace_max_z: float = 0.80,
        ee_orientation_frame: Optional[str] = None,
        use_current_ee_orientation: bool = True,
    ):
        super().__init__("PrepareDetectedPickPoses")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.pregrasp_offset_z_m = float(pregrasp_offset_z_m)
        self.grasp_offset_z_m = float(grasp_offset_z_m)
        self.lift_offset_z_m = float(lift_offset_z_m)
        self.workspace_min_x = float(workspace_min_x)
        self.workspace_max_x = float(workspace_max_x)
        self.workspace_min_y = float(workspace_min_y)
        self.workspace_max_y = float(workspace_max_y)
        self.workspace_min_z = float(workspace_min_z)
        self.workspace_max_z = float(workspace_max_z)
        self.ee_orientation_frame = (
            str(ee_orientation_frame).strip() if ee_orientation_frame is not None else None
        )
        self.use_current_ee_orientation = bool(use_current_ee_orientation)
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None

    def _config_float(self, bb_attr: str, fallback: float) -> float:
        return float(getattr(self.bb, bb_attr, fallback))

    def setup(self, **kwargs):
        if getattr(self.bb, "shared_tf_buffer", None) is not None:
            return True
        if not self.use_current_ee_orientation:
            return True
        if self.tf_listener is not None:
            return True
        if not rclpy.ok():
            return False
        node_name = f"bt_pick_pose_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        return True

    def _clamp_z(self, value: float) -> float:
        min_z = self._config_float("viperx_workspace_min_z", self.workspace_min_z)
        max_z = self._config_float("viperx_workspace_max_z", self.workspace_max_z)
        return min(max(float(value), min_z), max_z)

    def _within_workspace(self, x: float, y: float, z: float) -> bool:
        return (
            self._config_float("viperx_workspace_min_x", self.workspace_min_x)
            <= float(x)
            <= self._config_float("viperx_workspace_max_x", self.workspace_max_x)
            and self._config_float("viperx_workspace_min_y", self.workspace_min_y)
            <= float(y)
            <= self._config_float("viperx_workspace_max_y", self.workspace_max_y)
            and self._config_float("viperx_workspace_min_z", self.workspace_min_z)
            <= float(z)
            <= self._config_float("viperx_workspace_max_z", self.workspace_max_z)
        )

    @staticmethod
    def _copy_pose(
        detected_pose: dict,
        *,
        z: float,
        use_cartesian_approach: bool,
        quat_xyzw: tuple[float, float, float, float],
    ) -> dict:
        pose = {
            "frame_id": str(detected_pose.get("frame_id", "vx300s/base_link")),
            "x": float(detected_pose.get("x", 0.0)),
            "y": float(detected_pose.get("y", 0.0)),
            "z": float(z),
            "qx": float(quat_xyzw[0]),
            "qy": float(quat_xyzw[1]),
            "qz": float(quat_xyzw[2]),
            "qw": float(quat_xyzw[3]),
            "use_cartesian_approach": bool(use_cartesian_approach),
        }
        return pose

    def _resolve_orientation_frame(self) -> str:
        if self.ee_orientation_frame:
            return self.ee_orientation_frame
        frame = str(getattr(self.bb, "viperx_ee_orientation_frame", "")).strip()
        return frame or "vx300s/ee_gripper_link"

    def _resolve_ee_link(self) -> str:
        ee_link = str(getattr(self.bb, "viperx_ee_link", "")).strip()
        if ee_link:
            return ee_link
        return self._resolve_orientation_frame()

    def _get_pick_orientation_xyzw(self) -> tuple[float, float, float, float]:
        configured = getattr(self.bb, "viperx_fixed_pick_orientation_xyzw", None)
        if (
            isinstance(configured, (list, tuple))
            and len(configured) == 4
        ):
            return tuple(float(v) for v in configured)

        locked = getattr(self.bb, "locked_pick_orientation_xyzw", None)
        if (
            isinstance(locked, (list, tuple))
            and len(locked) == 4
        ):
            return tuple(float(v) for v in locked)

        use_current = bool(
            getattr(self.bb, "viperx_use_current_ee_orientation", self.use_current_ee_orientation)
        )
        tf_buffer = getattr(self.bb, "shared_tf_buffer", None) or self.tf_buffer
        if not use_current or tf_buffer is None:
            return (0.0, 0.0, 0.0, 1.0)

        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        try:
            transform = tf_buffer.lookup_transform(
                _resolve_base_frame(self.bb),
                self._resolve_orientation_frame(),
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            self.feedback_message = (
                "Unable to lock current EE orientation for pick poses: "
                f"{exc}"
            )
            return (0.0, 0.0, 0.0, 1.0)

        q = transform.transform.rotation
        quat = (float(q.x), float(q.y), float(q.z), float(q.w))
        self.bb.locked_pick_orientation_xyzw = quat
        return quat

    def update(self):
        detected_pose = getattr(self.bb, "detected_object_pose", None)
        if not isinstance(detected_pose, dict):
            self.feedback_message = "No detected object pose available for pick planning"
            return py_trees.common.Status.FAILURE

        x = float(detected_pose.get("x", 0.0))
        y = float(detected_pose.get("y", 0.0))
        z = float(detected_pose.get("z", 0.0))

        if not self._within_workspace(x, y, z):
            self.feedback_message = (
                "Detected pose outside legacy workspace bounds: "
                f"x={x:.3f} y={y:.3f} z={z:.3f}"
            )
            return py_trees.common.Status.FAILURE

        grasp_z = self._clamp_z(
            z + self._config_float("viperx_grasp_offset_z_m", self.grasp_offset_z_m)
        )
        pregrasp_z = self._clamp_z(
            grasp_z + self._config_float("viperx_pregrasp_offset_z_m", self.pregrasp_offset_z_m)
        )
        lift_z = self._clamp_z(
            grasp_z + self._config_float("viperx_lift_offset_z_m", self.lift_offset_z_m)
        )
        quat_xyzw = self._get_pick_orientation_xyzw()

        self.bb.pregrasp_pose = self._copy_pose(
            detected_pose,
            z=pregrasp_z,
            use_cartesian_approach=False,
            quat_xyzw=quat_xyzw,
        )
        self.bb.pregrasp_pose["ee_link"] = self._resolve_ee_link()
        self.bb.grasp_pose = self._copy_pose(
            detected_pose,
            z=grasp_z,
            use_cartesian_approach=True,
            quat_xyzw=quat_xyzw,
        )
        self.bb.grasp_pose["ee_link"] = self._resolve_ee_link()
        self.bb.grasp_pose["gripper_close_position"] = float(
            getattr(self.bb, "viperx_close_gripper_position", 0.0)
        )
        self.bb.lift_pose = self._copy_pose(
            detected_pose,
            z=lift_z,
            use_cartesian_approach=True,
            quat_xyzw=quat_xyzw,
        )
        self.bb.lift_pose["ee_link"] = self._resolve_ee_link()

        self.feedback_message = (
            "Prepared pick poses "
            f"pregrasp_z={pregrasp_z:.3f} grasp_z={grasp_z:.3f} lift_z={lift_z:.3f}"
        )
        return py_trees.common.Status.SUCCESS

    def shutdown(self):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None


class MoveViperXGripper(py_trees.behaviour.Behaviour):
    """
    Command ViperX gripper through PickArm action:
      - planning_group = open_gripper / close_gripper
    """

    def __init__(self, command: str, bb=None, action_name: str = "/pick_viperx"):
        super().__init__(f"MoveViperXGripper[{command}]")
        self.command = command
        self.action_name = action_name
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.client: Optional[ActionClient] = None
        self.node = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def setup(self, **kwargs):
        if self.client is not None:
            return True
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
        goal_msg.target_pose.header.frame_id = _resolve_base_frame(self.bb)
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
            result_msg = ""
            if getattr(wrapped, "result", None) is not None:
                result_msg = str(getattr(wrapped.result, "message", "") or "")
            self.feedback_message = (
                f"Gripper failed with status={wrapped.status}"
                + (f": {result_msg}" if result_msg else "")
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        del new_status
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def shutdown(self):
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
        else:
            # Use random items slot (slot 3)
            selected_slot = 3
        selected_pose = basket_poses[selected_slot]

        if selected_pose is None:
            self.feedback_message = (
                f"Basket pose not initialized at slot {selected_slot}"
            )
            return py_trees.common.Status.FAILURE

        self.bb.basket_pose = deepcopy(selected_pose)
        if is_bottle:
            self.bb.basket_bottle_count = bottle_count + 1
            self.feedback_message = f"Selected bottle slot {selected_slot + 1}/3"
        else:
            self.feedback_message = "Selected random items slot"

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
        if self.client is not None:
            return True
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
        pose_msg.header.frame_id = target_pose.get(
            "frame_id",
            _resolve_base_frame(self.bb),
        )
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
        self.send_future = None
        self.goal_handle = None
        self.result_future = None

    def shutdown(self):
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
