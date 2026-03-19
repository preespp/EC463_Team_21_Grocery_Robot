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
        self.declare_parameter("dry_run", False)
        self.declare_parameter("fixed_orientation_xyzw", "0.0,0.0,0.0,1.0")

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_link = str(self.get_parameter("ee_link").value)
        self.ee_orientation_frame = str(self.get_parameter("ee_orientation_frame").value)
        self.use_current_ee_orientation = bool(self.get_parameter("use_current_ee_orientation").value)
        self.target_class = str(self.get_parameter("target_class").value).strip().lower()
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
        self.dry_run = bool(self.get_parameter("dry_run").value)
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
        self.stable_count = 0

        self.sequence: List[Tuple[str, PickArm.Goal]] = []
        self.step_index = 0
        self.state = "idle"
        self.send_future = None
        self.result_future = None
        self.next_allowed_pick_time = self.get_clock().now()
        self.last_tf_error_log_time = self.get_clock().now()

        self.get_logger().info(
            "vision_auto_pick started: "
            f"detections_topic={self.detections_topic} "
            f"action_name={self.action_name} "
            f"base_frame={self.base_frame} "
            f"target_class={self.target_class if self.target_class else '<any>'} "
            f"enable_auto_pick={self.enable_auto_pick} "
            f"dry_run={self.dry_run}"
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
    def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)

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
            return

        best = self._select_best_detection(detections, camera_frame)
        if best is None:
            self.stable_count = 0
            self.last_stable_xyz = None
            return

        xyz = best["xyz"]
        if self.last_stable_xyz is not None and self._distance(self.last_stable_xyz, xyz) <= self.target_match_distance_m:
            self.stable_count += 1
            self.last_stable_xyz = xyz
        else:
            self.stable_count = 1
            self.last_stable_xyz = xyz

        if self.stable_count < self.target_stability_count:
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
    ) -> Optional[Dict[str, object]]:
        candidates: List[Dict[str, object]] = []
        for det in detections:
            if not isinstance(det, dict):
                continue
            cls_name = str(det.get("class_name", "")).strip().lower()
            if self.target_class and cls_name != self.target_class:
                continue

            confidence = float(det.get("confidence", 0.0))
            if confidence < self.min_confidence:
                continue

            depth_m = float(det.get("distance_m", 0.0))
            if depth_m <= 0.0 or depth_m < self.min_depth_m or depth_m > self.max_depth_m:
                continue

            point_m = det.get("point_camera_optical_m", [])
            if not isinstance(point_m, list) or len(point_m) != 3:
                continue

            transformed = self._transform_point_to_base(point_m, camera_frame)
            if transformed is None:
                continue
            if not self._within_workspace(transformed):
                continue

            candidates.append({
                "xyz": transformed,
                "class_name": cls_name or "unknown",
                "confidence": confidence,
                "depth_m": depth_m,
            })

        if not candidates:
            return None
        candidates.sort(key=lambda c: (float(c["depth_m"]), -float(c["confidence"])))
        return candidates[0]

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
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_orientation_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
            q = transform.transform.rotation
            return float(q.x), float(q.y), float(q.z), float(q.w)
        except TransformException:
            return self.fixed_orientation_xyzw

    def _make_arm_goal(
        self,
        xyz: Tuple[float, float, float],
        quat_xyzw: Tuple[float, float, float, float],
        close_position: float = -1.0,
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
        goal.use_cartesian_approach = False
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
            ("move_pregrasp", self._make_arm_goal(pre_xyz, q, close_position=-1.0)),
            ("move_grasp_and_close", self._make_arm_goal(grasp_xyz, q, close_position=self.close_gripper_position)),
            ("lift", self._make_arm_goal(lift_xyz, q, close_position=-1.0)),
        ]

        if self.put_back_after_pick:
            place_z = min(
                max(grasp_z + self.place_offset_z_m, self.workspace_min_z),
                self.workspace_max_z,
            )
            place_xyz = (xyz[0], xyz[1], place_z)
            sequence.extend([
                ("move_back_to_pregrasp", self._make_arm_goal(pre_xyz, q, close_position=-1.0)),
                ("move_place", self._make_arm_goal(place_xyz, q, close_position=-1.0)),
                ("open_gripper_place", self._make_gripper_goal("open_gripper")),
                ("retreat_after_place", self._make_arm_goal(pre_xyz, q, close_position=-1.0)),
            ])

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
