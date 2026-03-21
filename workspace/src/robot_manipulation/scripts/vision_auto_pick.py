#!/usr/bin/env python3

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from robot_interfaces.action import PickArm
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401 - required to register PointStamped transforms
from tf2_ros import Buffer, TransformException, TransformListener


class VisionAutoPick(Node):
    """
    Bridge camera detections to the ViperX PickArm action server.
    """

    def __init__(self) -> None:
        super().__init__("vision_auto_pick")

        self.declare_parameter("detections_topic", "/detections_json")
        self.declare_parameter("action_name", "/pick_viperx")
        self.declare_parameter("base_frame", "vx300s/base_link")
        self.declare_parameter("ee_link", "")
        self.declare_parameter("ee_orientation_frame", "vx300s/ee_gripper_link")
        self.declare_parameter("use_current_ee_orientation", True)
        self.declare_parameter("target_class", "")
        self.declare_parameter("min_confidence", 0.60)
        self.declare_parameter("min_depth_m", 0.08)
        self.declare_parameter("max_depth_m", 0.70)
        self.declare_parameter("workspace_min_x", 0.12)
        self.declare_parameter("workspace_max_x", 0.52)
        self.declare_parameter("workspace_min_y", -0.30)
        self.declare_parameter("workspace_max_y", 0.30)
        self.declare_parameter("workspace_min_z", 0.00)
        self.declare_parameter("workspace_max_z", 0.35)
        self.declare_parameter("pregrasp_offset_z_m", 0.08)
        self.declare_parameter("grasp_offset_z_m", 0.00)
        self.declare_parameter("lift_offset_z_m", 0.12)
        self.declare_parameter("place_offset_z_m", 0.00)
        self.declare_parameter("close_gripper_position", 0.0)
        self.declare_parameter("put_back_after_pick", True)
        self.declare_parameter("cooldown_sec", 3.0)
        self.declare_parameter("target_stability_count", 4)
        self.declare_parameter("target_match_distance_m", 0.03)
        self.declare_parameter("goal_timeout_sec", 15.0)
        self.declare_parameter("pick_once", False)
        self.declare_parameter("enable_auto_pick", True)
        self.declare_parameter("return_after_sequence", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("fixed_orientation_xyzw", "0.0,0.0,0.0,1.0")
        self.declare_parameter("lock_initial_ee_orientation", True)
        self.declare_parameter("diagnostics_log_period_sec", 2.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.ee_orientation_frame = str(self.get_parameter("ee_orientation_frame").value)
        self.use_current_ee_orientation = bool(self.get_parameter("use_current_ee_orientation").value)
        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
        self.target_classes = self._parse_target_classes(self.target_class)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.workspace_min_x = float(self.get_parameter("workspace_min_x").value)
        self.workspace_max_x = float(self.get_parameter("workspace_max_x").value)
        self.workspace_min_y = float(self.get_parameter("workspace_min_y").value)
        self.workspace_max_y = float(self.get_parameter("workspace_max_y").value)
        self.workspace_min_z = float(self.get_parameter("workspace_min_z").value)
        self.workspace_max_z = float(self.get_parameter("workspace_max_z").value)
        self.pregrasp_offset_z_m = float(self.get_parameter("pregrasp_offset_z_m").value)
        self.grasp_offset_z_m = float(self.get_parameter("grasp_offset_z_m").value)
        self.lift_offset_z_m = float(self.get_parameter("lift_offset_z_m").value)
        self.place_offset_z_m = float(self.get_parameter("place_offset_z_m").value)
        self.close_gripper_position = float(self.get_parameter("close_gripper_position").value)
        self.put_back_after_pick = bool(self.get_parameter("put_back_after_pick").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.target_stability_count = int(self.get_parameter("target_stability_count").value)
        self.target_match_distance_m = float(self.get_parameter("target_match_distance_m").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.pick_once = bool(self.get_parameter("pick_once").value)
        self.enable_auto_pick = bool(self.get_parameter("enable_auto_pick").value)
        self.return_after_sequence = bool(self.get_parameter("return_after_sequence").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.lock_initial_ee_orientation = bool(
            self.get_parameter("lock_initial_ee_orientation").value
        )
        self.diagnostics_log_period_sec = float(
            self.get_parameter("diagnostics_log_period_sec").value
        )
        self.fixed_orientation_xyzw = self._parse_quat(
            str(self.get_parameter("fixed_orientation_xyzw").value)
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, PickArm, self.action_name)

        self.sub_detections = self.create_subscription(
            String,
            self.detections_topic,
            self._on_detections,
            10,
        )
        self.step_timer = self.create_timer(0.05, self._tick_action_state)

        self.pending_pick_xyz: Optional[Tuple[float, float, float]] = None
        self.pending_pick_meta: Optional[Dict[str, float]] = None
        self.last_stable_xyz: Optional[Tuple[float, float, float]] = None
        self.locked_pick_orientation_xyzw: Optional[Tuple[float, float, float, float]] = None
        self.stable_count = 0

        self.sequence: List[Tuple[str, PickArm.Goal]] = []
        self.step_index = 0
        self.state = "idle"
        self.send_future = None
        self.result_future = None
        self.next_allowed_pick_time = self.get_clock().now()
        self.last_tf_error_log_time = self.get_clock().now()
        self.last_diagnostics_log_time = self.get_clock().now()
        self.last_detection_msg_time = None

        self.get_logger().info(
            "vision_auto_pick started: "
            f"detections_topic={self.detections_topic} "
            f"action_name={self.action_name} "
            f"base_frame={self.base_frame} "
            f"target_class={','.join(sorted(self.target_classes)) if self.target_classes else '<any>'} "
            f"enable_auto_pick={self.enable_auto_pick} "
            f"dry_run={self.dry_run}"
        )
        self.get_logger().info(
            "vision_auto_pick filters: "
            f"min_conf={self.min_confidence:.2f} "
            f"depth_range=({self.min_depth_m:.2f}, {self.max_depth_m:.2f}) "
            f"workspace_x=({self.workspace_min_x:.2f}, {self.workspace_max_x:.2f}) "
            f"workspace_y=({self.workspace_min_y:.2f}, {self.workspace_max_y:.2f}) "
            f"workspace_z=({self.workspace_min_z:.2f}, {self.workspace_max_z:.2f}) "
            f"stability={self.target_stability_count}"
        )

    @staticmethod
    def _parse_quat(text: str) -> Tuple[float, float, float, float]:
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 4:
            raise ValueError(f"Expected 4 comma-separated values for quaternion, got: {text}")
        q = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
        norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if norm <= 1e-8:
            return 0.0, 0.0, 0.0, 1.0
        return q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm

    @staticmethod
    def _parse_target_classes(text: str) -> set[str]:
        return {
            part.strip().lower()
            for part in text.split(",")
            if part.strip()
        }

    @staticmethod
    def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

    @staticmethod
    def _format_xyz(xyz: Tuple[float, float, float]) -> str:
        return f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"

    def _should_log_diagnostics(self, now) -> bool:
        if self.diagnostics_log_period_sec <= 0.0:
            return False
        return (
            now - self.last_diagnostics_log_time
        ).nanoseconds >= int(self.diagnostics_log_period_sec * 1e9)

    def _maybe_log_diagnostics(self, message: str) -> None:
        now = self.get_clock().now()
        if not self._should_log_diagnostics(now):
            return
        self.last_diagnostics_log_time = now
        self.get_logger().info(message)

    def _within_workspace(self, xyz: Tuple[float, float, float]) -> bool:
        x, y, z = xyz
        return (
            self.workspace_min_x <= x <= self.workspace_max_x
            and self.workspace_min_y <= y <= self.workspace_max_y
            and self.workspace_min_z <= z <= self.workspace_max_z
        )

    def _on_detections(self, msg: String) -> None:
        if not self.enable_auto_pick or self.state != "idle" or self.pending_pick_xyz is not None:
            return

        now = self.get_clock().now()
        self.last_detection_msg_time = now
        if now < self.next_allowed_pick_time:
            return

        try:
            payload = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Ignoring invalid detections_json payload: {exc}")
            return

        camera_frame = str(payload.get("camera_optical_frame", "")).strip()
        if not camera_frame:
            self.get_logger().warn("Ignoring detections_json payload without camera_optical_frame")
            return
        detections = payload.get("detections", [])
        if not isinstance(detections, list) or not detections:
            self.stable_count = 0
            self.last_stable_xyz = None
            self._maybe_log_diagnostics(
                f"Auto-pick sees camera stream from {camera_frame}, but detections list is empty."
            )
            return

        best, summary = self._select_best_detection(detections, camera_frame)
        if best is None:
            self.stable_count = 0
            self.last_stable_xyz = None
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
                message += f" example_base_xyz={self._format_xyz(example_xyz)}"
            self._maybe_log_diagnostics(message)
            return

        xyz = best["xyz"]
        if self.last_stable_xyz is not None and self._distance(self.last_stable_xyz, xyz) <= self.target_match_distance_m:
            self.stable_count += 1
            self.last_stable_xyz = xyz
        else:
            self.stable_count = 1
            self.last_stable_xyz = xyz

        if self.stable_count < self.target_stability_count:
            self._maybe_log_diagnostics(
                "Candidate accepted but waiting for stability: "
                f"{self.stable_count}/{self.target_stability_count} "
                f"class={best['class_name']} "
                f"conf={float(best['confidence']):.2f} "
                f"base_xyz={self._format_xyz(xyz)}"
            )
            return

        self.pending_pick_xyz = xyz
        self.pending_pick_meta = best
        self.stable_count = 0
        self.last_stable_xyz = None
        self.get_logger().info(
            "Queued pick target "
            f"class={best['class_name']} "
            f"conf={float(best['confidence']):.2f} "
            f"base_xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})"
        )

    def _select_best_detection(
        self,
        detections: List[dict],
        camera_frame: str,
    ) -> Tuple[Optional[Dict[str, object]], Dict[str, object]]:
        candidates: List[Dict[str, object]] = []
        summary: Dict[str, object] = {
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
            if self.target_classes and cls_name not in self.target_classes:
                summary["class_mismatch"] = int(summary["class_mismatch"]) + 1
                continue

            confidence = float(det.get("confidence", 0.0))
            if confidence < self.min_confidence:
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

            transformed = self._transform_point_to_base(point_m, camera_frame)
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
            })

        if not candidates:
            return None, summary
        candidates.sort(key=lambda c: (float(c["depth_m"]), -float(c["confidence"])))
        return candidates[0], summary

    def _transform_point_to_base(
        self,
        point_m: List[float],
        source_frame: str,
    ) -> Optional[Tuple[float, float, float]]:
        point = PointStamped()
        # Use latest available transform to avoid frequent time sync/extrapolation misses.
        point.header.stamp.sec = 0
        point.header.stamp.nanosec = 0
        point.header.frame_id = source_frame
        point.point.x = float(point_m[0])
        point.point.y = float(point_m[1])
        point.point.z = float(point_m[2])
        try:
            transformed = self.tf_buffer.transform(
                point,
                self.base_frame,
                timeout=Duration(seconds=0.25),
            )
        except TransformException as exc:
            now = self.get_clock().now()
            if (now - self.last_tf_error_log_time).nanoseconds > int(2.0 * 1e9):
                self.last_tf_error_log_time = now
                self.get_logger().warn(
                    f"TF transform failed source={source_frame} target={self.base_frame}: {exc}"
                )
            return None

        return (
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        )

    def _get_pick_orientation(self) -> Tuple[float, float, float, float]:
        if not self.use_current_ee_orientation:
            return self.fixed_orientation_xyzw
        if self.lock_initial_ee_orientation and self.locked_pick_orientation_xyzw is not None:
            return self.locked_pick_orientation_xyzw
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_orientation_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
            q = transform.transform.rotation
            quat = (float(q.x), float(q.y), float(q.z), float(q.w))
            if self.lock_initial_ee_orientation and self.locked_pick_orientation_xyzw is None:
                self.locked_pick_orientation_xyzw = quat
                self.get_logger().info(
                    "Locked pick orientation from current EE pose: "
                    f"xyzw=({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})"
                )
            return quat
        except TransformException:
            return self.fixed_orientation_xyzw

    def _make_arm_goal(
        self,
        xyz: Tuple[float, float, float],
        quat_xyzw: Tuple[float, float, float, float],
        close_position: float = -1.0,
        use_cartesian: bool = False,
    ) -> PickArm.Goal:
        goal = PickArm.Goal()
        goal.target_pose.header.frame_id = self.base_frame
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = float(xyz[0])
        goal.target_pose.pose.position.y = float(xyz[1])
        goal.target_pose.pose.position.z = float(xyz[2])
        goal.target_pose.pose.orientation.x = float(quat_xyzw[0])
        goal.target_pose.pose.orientation.y = float(quat_xyzw[1])
        goal.target_pose.pose.orientation.z = float(quat_xyzw[2])
        goal.target_pose.pose.orientation.w = float(quat_xyzw[3])
        goal.planning_group = "arm"
        goal.ee_link = self.ee_link
        goal.pregrasp_offset_m = 0.0
        goal.retreat_offset_m = 0.0
        goal.gripper_close_position = float(close_position)
        goal.use_cartesian_approach = bool(use_cartesian)
        return goal

    def _make_gripper_goal(self, command: str) -> PickArm.Goal:
        goal = PickArm.Goal()
        goal.target_pose.header.frame_id = self.base_frame
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.orientation.w = 1.0
        goal.planning_group = command
        goal.ee_link = ""
        goal.pregrasp_offset_m = 0.0
        goal.retreat_offset_m = 0.0
        goal.gripper_close_position = -1.0
        goal.use_cartesian_approach = False
        return goal

    def _make_command_goal(self, command: str) -> PickArm.Goal:
        goal = PickArm.Goal()
        goal.target_pose.header.frame_id = self.base_frame
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.orientation.w = 1.0
        goal.planning_group = command
        goal.ee_link = ""
        goal.pregrasp_offset_m = 0.0
        goal.retreat_offset_m = 0.0
        goal.gripper_close_position = -1.0
        goal.use_cartesian_approach = False
        return goal

    def _build_pick_sequence(self, xyz: Tuple[float, float, float]) -> List[Tuple[str, PickArm.Goal]]:
        q = self._get_pick_orientation()
        grasp_z = min(max(xyz[2] + self.grasp_offset_z_m, self.workspace_min_z), self.workspace_max_z)
        pre_z = min(max(grasp_z + self.pregrasp_offset_z_m, self.workspace_min_z), self.workspace_max_z)
        lift_z = min(max(grasp_z + self.lift_offset_z_m, self.workspace_min_z), self.workspace_max_z)

        pre_xyz = (xyz[0], xyz[1], pre_z)
        grasp_xyz = (xyz[0], xyz[1], grasp_z)
        lift_xyz = (xyz[0], xyz[1], lift_z)

        sequence = [
            ("open_gripper", self._make_gripper_goal("open_gripper")),
            ("move_pregrasp", self._make_arm_goal(pre_xyz, q, close_position=-1.0, use_cartesian=False)),
            (
                "move_grasp_and_close",
                self._make_arm_goal(
                    grasp_xyz,
                    q,
                    close_position=self.close_gripper_position,
                    use_cartesian=True,
                ),
            ),
            ("lift", self._make_arm_goal(lift_xyz, q, close_position=-1.0, use_cartesian=True)),
        ]

        if self.put_back_after_pick:
            sequence.extend([
                ("move_to_place_pose", self._make_command_goal("place_arm_pose")),
                ("open_gripper_place", self._make_gripper_goal("open_gripper")),
            ])

        if self.return_after_sequence:
            sequence.append(("return_arm_pose", self._make_command_goal("return_arm_pose")))

        return sequence

    def _start_next_step(self) -> None:
        if self.step_index >= len(self.sequence):
            self._finish_sequence(success=True, message="Auto-pick sequence complete")
            return

        label, goal = self.sequence[self.step_index]
        if self.dry_run:
            self.get_logger().info(f"[dry_run] step={label} goal sent virtually")
            self.step_index += 1
            return

        if not self.action_client.wait_for_server(timeout_sec=0.2):
            self._finish_sequence(False, f"Action server unavailable: {self.action_name}")
            return

        self.get_logger().info(
            f"Auto-pick step {self.step_index + 1}/{len(self.sequence)}: {label}"
        )
        self.send_future = self.action_client.send_goal_async(goal)
        self.result_future = None
        self.state = "waiting_goal"

    def _finish_sequence(self, success: bool, message: str) -> None:
        self.sequence = []
        self.step_index = 0
        self.send_future = None
        self.result_future = None
        self.state = "idle"
        self.next_allowed_pick_time = self.get_clock().now() + Duration(seconds=self.cooldown_sec)

        if success:
            self.get_logger().info(message)
            if self.pick_once:
                self.enable_auto_pick = False
                self.get_logger().info("pick_once=true -> auto-pick disabled after successful pick")
        else:
            self.get_logger().warn(message)

    def _tick_action_state(self) -> None:
        if self.enable_auto_pick and self.state == "idle":
            now = self.get_clock().now()
            if self.last_detection_msg_time is None:
                self._maybe_log_diagnostics(
                    f"No detections_json messages received yet on {self.detections_topic}. "
                    "Check that camera_vision is running."
                )
            elif (
                now - self.last_detection_msg_time
            ).nanoseconds >= int(max(5.0, self.diagnostics_log_period_sec) * 1e9):
                self._maybe_log_diagnostics(
                    f"No new detections_json messages for "
                    f"{(now - self.last_detection_msg_time).nanoseconds / 1e9:.1f}s "
                    f"on {self.detections_topic}."
                )

        if self.state == "idle":
            if self.pending_pick_xyz is None:
                return
            target = self.pending_pick_xyz
            meta = self.pending_pick_meta or {}
            self.pending_pick_xyz = None
            self.pending_pick_meta = None
            self.sequence = self._build_pick_sequence(target)
            self.step_index = 0
            self.state = "running"
            self.get_logger().info(
                "Starting auto-pick sequence at "
                f"xyz=({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}), "
                f"class={str(meta.get('class_name', 'unknown'))} "
                f"conf={float(meta.get('confidence', 0.0)):.2f}"
            )
            return

        if self.state == "running":
            self._start_next_step()
            return

        if self.state == "waiting_goal":
            if self.send_future is None or not self.send_future.done():
                return
            goal_handle = self.send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._finish_sequence(False, "Auto-pick goal rejected")
                return
            self.result_future = goal_handle.get_result_async()
            self.state = "waiting_result"
            return

        if self.state == "waiting_result":
            if self.result_future is None or not self.result_future.done():
                return
            wrapped_result = self.result_future.result()
            if wrapped_result is None:
                self._finish_sequence(False, "Auto-pick action returned no result")
                return
            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
                self._finish_sequence(False, f"Auto-pick action failed with status={wrapped_result.status}")
                return

            result = wrapped_result.result
            if not result.success:
                self._finish_sequence(False, f"Auto-pick step failed: {result.message}")
                return

            self.step_index += 1
            self.state = "running"

def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionAutoPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
