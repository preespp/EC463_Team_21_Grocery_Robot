#!/usr/bin/env python3
"""
Mission orchestrator skeleton (P1):

BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY

This node coordinates Nav2 actions and Cartographer map persistence while supporting
manual override preemption and resume.
"""

from __future__ import annotations

import math
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from cartographer_ros_msgs.srv import WriteState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.utilities import ok as rclpy_ok
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here.parent, *here.parents]:
        if (
            (parent / "Nav").exists()
            and (parent / "Maps").exists()
            and (parent / "workspace").exists()
        ):
            return parent
    cwd = Path.cwd()
    if (cwd / "Nav").exists() and (cwd / "Maps").exists():
        return cwd
    return cwd


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _normalize_namespace(namespace: str) -> str:
    cleaned = namespace.strip()
    if not cleaned:
        return ""
    if not cleaned.startswith("/"):
        cleaned = "/" + cleaned
    return cleaned.rstrip("/")


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class MissionOrchestrator(Node):
    """State machine for fixed-trajectory mapping mission (P1)."""

    STATE_IDLE = "IDLE"
    STATE_BOOT = "BOOT"
    STATE_AUTO_MAP_V1 = "AUTO_MAP_V1"
    STATE_AUTO_EXPLORE = "AUTO_EXPLORE"
    STATE_RETURN_HOME = "RETURN_HOME"
    STATE_SAVE_EXPORT = "SAVE_EXPORT"
    STATE_LOCALIZE_READY = "LOCALIZE_READY"
    STATE_PAUSED = "PAUSED"
    STATE_ERROR = "ERROR"

    def __init__(self) -> None:
        super().__init__("mission_orchestrator")

        repo_root = _find_repo_root()
        default_maps_dir = str(repo_root / "Maps")

        self.declare_parameter("autostart", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("nav2_namespace", "")
        self.declare_parameter("manual_override_topic", "/manual_override")
        self.declare_parameter("follow_waypoints_action", "/follow_waypoints")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("write_state_service", "/write_state")
        self.declare_parameter("state_topic", "/mission_state")
        self.declare_parameter("loop_rate_hz", 10.0)
        self.declare_parameter("mapping_mode", "fixed")  # fixed | frontier

        self.declare_parameter("boot_capture_delay_sec", 2.0)
        self.declare_parameter("mapping_timeout_sec", 180.0)
        self.declare_parameter("mapping_max_distance_m", 80.0)
        self.declare_parameter("mapping_loop_pause_sec", 0.4)
        self.declare_parameter("mapping_retry_wait_sec", 1.0)
        self.declare_parameter("mapping_max_failures", 3)
        self.declare_parameter("explore_timeout_sec", 20.0 * 60.0)

        self.declare_parameter("frontier_done_topic", "/frontier_explorer/done")
        self.declare_parameter("frontier_start_service", "/frontier_explorer/start")
        self.declare_parameter("frontier_stop_service", "/frontier_explorer/stop")

        self.declare_parameter("home_retry_limit", 2)
        self.declare_parameter("action_server_timeout_sec", 0.3)

        self.declare_parameter("map_output_dir", default_maps_dir)
        self.declare_parameter("map_name", "")
        self.declare_parameter("map_name_prefix", "run")
        self.declare_parameter("include_unfinished_submaps", True)
        self.declare_parameter("export_map", True)
        self.declare_parameter("export_resolution", 0.03)

        self.declare_parameter(
            "auto_map_waypoints",
            [
                1.0,
                0.0,
                0.0,
                1.0,
                0.8,
                1.57,
                0.0,
                0.8,
                3.14,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.declare_parameter("append_home_waypoint", True)

        self.autostart = bool(self.get_parameter("autostart").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.nav2_namespace = _normalize_namespace(
            str(self.get_parameter("nav2_namespace").value)
        )
        self.manual_override_topic = str(self.get_parameter("manual_override_topic").value)
        self.follow_waypoints_action = self._resolve_nav2_action_name(
            str(self.get_parameter("follow_waypoints_action").value)
        )
        self.navigate_to_pose_action = self._resolve_nav2_action_name(
            str(self.get_parameter("navigate_to_pose_action").value)
        )
        self.write_state_service = str(self.get_parameter("write_state_service").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.loop_rate_hz = max(1e-3, float(self.get_parameter("loop_rate_hz").value))
        self.mapping_mode = str(self.get_parameter("mapping_mode").value).strip().lower()
        if self.mapping_mode not in ("fixed", "frontier"):
            self.get_logger().warning(
                f"Unknown mapping_mode='{self.mapping_mode}', fallback to 'fixed'"
            )
            self.mapping_mode = "fixed"

        self.boot_capture_delay_sec = max(
            0.0, float(self.get_parameter("boot_capture_delay_sec").value)
        )
        self.mapping_timeout_sec = max(
            0.0, float(self.get_parameter("mapping_timeout_sec").value)
        )
        self.mapping_max_distance_m = max(
            0.0, float(self.get_parameter("mapping_max_distance_m").value)
        )
        self.mapping_loop_pause_sec = max(
            0.0, float(self.get_parameter("mapping_loop_pause_sec").value)
        )
        self.mapping_retry_wait_sec = max(
            0.0, float(self.get_parameter("mapping_retry_wait_sec").value)
        )
        self.mapping_max_failures = max(
            1, int(self.get_parameter("mapping_max_failures").value)
        )
        self.explore_timeout_sec = max(
            10.0, float(self.get_parameter("explore_timeout_sec").value)
        )
        self.home_retry_limit = max(0, int(self.get_parameter("home_retry_limit").value))
        self.action_server_timeout_sec = max(
            0.05, float(self.get_parameter("action_server_timeout_sec").value)
        )
        self.frontier_done_topic = str(self.get_parameter("frontier_done_topic").value)
        self.frontier_start_service = str(self.get_parameter("frontier_start_service").value)
        self.frontier_stop_service = str(self.get_parameter("frontier_stop_service").value)

        self.map_output_dir = Path(str(self.get_parameter("map_output_dir").value))
        self.map_name_fixed = str(self.get_parameter("map_name").value).strip()
        self.map_name_prefix = str(self.get_parameter("map_name_prefix").value).strip() or "run"
        self.include_unfinished_submaps = bool(
            self.get_parameter("include_unfinished_submaps").value
        )
        self.export_map = bool(self.get_parameter("export_map").value)
        self.export_resolution = float(self.get_parameter("export_resolution").value)
        self.append_home_waypoint = bool(self.get_parameter("append_home_waypoint").value)
        self.auto_map_waypoint_offsets = self._parse_waypoint_offsets(
            self.get_parameter("auto_map_waypoints").value
        )

        qos = QoSProfile(depth=10)
        self.state_pub = self.create_publisher(String, self.state_topic, qos)
        self.manual_override_sub = self.create_subscription(
            Bool,
            self.manual_override_topic,
            self._on_manual_override,
            qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odom,
            qos,
        )
        self.frontier_done_sub = self.create_subscription(
            Bool,
            self.frontier_done_topic,
            self._on_frontier_done,
            qos,
        )

        self.start_srv = self.create_service(Trigger, "start_mission", self._on_start_mission)
        self.pause_srv = self.create_service(Trigger, "pause_mission", self._on_pause_mission)
        self.resume_srv = self.create_service(Trigger, "resume_mission", self._on_resume_mission)
        self.finish_srv = self.create_service(
            Trigger, "finish_mapping", self._on_finish_mapping
        )

        self.follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            self.follow_waypoints_action,
        )
        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
        )
        self.write_state_client = self.create_client(WriteState, self.write_state_service)
        self.frontier_start_client = self.create_client(Trigger, self.frontier_start_service)
        self.frontier_stop_client = self.create_client(Trigger, self.frontier_stop_service)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.STATE_IDLE
        self.state_entered_at = time.monotonic()
        self.resume_state = self.STATE_IDLE
        self.pause_reason = ""
        self.manual_override_active = False

        self.home_pose: Pose2D | None = None
        self.map_name_current = ""
        self.finish_mapping_requested = False

        self.mapping_started_at: float | None = None
        self.mapping_distance_m = 0.0
        self.map_loop_count = 0
        self.map_failures = 0
        self.next_map_dispatch_at = 0.0
        self.last_odom_xy: tuple[float, float] | None = None
        self.explore_started_at: float | None = None
        self.frontier_started = False
        self.frontier_done = False

        self.return_home_attempts = 0

        self.action_token = 0
        self.action_inflight = False
        self.active_goal_handle = None
        self.action_result: tuple[str, bool, int, str] | None = None

        self.save_stage = "idle"
        self.write_future = None
        self.pbstream_path: Path | None = None
        self.map_filestem: Path | None = None
        self.export_process: subprocess.Popen | None = None

        self.last_tf_warn_at = 0.0
        self.last_server_warn_at = 0.0

        self.loop_timer = self.create_timer(1.0 / self.loop_rate_hz, self._tick)

        if self.autostart:
            self._start_new_mission()
        else:
            self._publish_state()

        self.get_logger().info(
            "mission_orchestrator ready "
            f"(autostart={self.autostart}, mapping_mode={self.mapping_mode}, "
            f"nav2_namespace={self.nav2_namespace or '/'}"
            f", "
            f"manual_override_topic={self.manual_override_topic}, "
            f"follow_waypoints={self.follow_waypoints_action}, navigate_to_pose={self.navigate_to_pose_action}, "
            f"write_state={self.write_state_service})"
        )

    def _resolve_nav2_action_name(self, action_name: str) -> str:
        cleaned = action_name.strip()
        if not cleaned:
            return cleaned
        if not self.nav2_namespace:
            return cleaned

        if cleaned.startswith("/") and cleaned not in (
            "/follow_waypoints",
            "/navigate_to_pose",
        ):
            return cleaned

        return f"{self.nav2_namespace}/{cleaned.lstrip('/')}"

    def _summarize_action_result(self, result_msg: object) -> str:
        if result_msg is None:
            return ""

        missed = getattr(result_msg, "missed_waypoints", None)
        if missed:
            return f"missed_waypoints={list(missed)}"

        for attr in ("error_msg", "message"):
            value = getattr(result_msg, attr, "")
            if value:
                return f"{attr}={value}"

        error_code = getattr(result_msg, "error_code", None)
        if error_code not in (None, 0):
            return f"error_code={error_code}"

        text = repr(result_msg)
        if text.startswith("<") and " object at " in text:
            return ""
        if len(text) > 180:
            return text[:177] + "..."
        return text

    def _parse_waypoint_offsets(self, raw: Sequence[float]) -> List[Pose2D]:
        values = [float(v) for v in raw]
        if len(values) < 3 or len(values) % 3 != 0:
            raise ValueError(
                "auto_map_waypoints must contain triples [x,y,yaw,...] and at least one triple."
            )
        offsets: List[Pose2D] = []
        for index in range(0, len(values), 3):
            offsets.append(
                Pose2D(
                    x=values[index],
                    y=values[index + 1],
                    yaw=values[index + 2],
                )
            )
        return offsets

    def _on_start_mission(self, _request, response: Trigger.Response) -> Trigger.Response:
        if self.state not in (self.STATE_IDLE, self.STATE_LOCALIZE_READY, self.STATE_ERROR):
            response.success = False
            response.message = f"Mission already active in state {self.state}"
            return response
        self._start_new_mission()
        response.success = True
        response.message = "Mission started"
        return response

    def _on_pause_mission(self, _request, response: Trigger.Response) -> Trigger.Response:
        if self.state == self.STATE_PAUSED:
            response.success = True
            response.message = f"Already paused ({self.pause_reason})"
            return response
        if self.state in (self.STATE_IDLE, self.STATE_LOCALIZE_READY, self.STATE_ERROR):
            response.success = False
            response.message = f"Cannot pause from state {self.state}"
            return response
        self._pause_mission("service_pause")
        response.success = True
        response.message = "Mission paused"
        return response

    def _on_resume_mission(self, _request, response: Trigger.Response) -> Trigger.Response:
        if self.state != self.STATE_PAUSED:
            response.success = False
            response.message = f"Mission is not paused (state={self.state})"
            return response
        if self.manual_override_active:
            response.success = False
            response.message = "manual_override is still true; cannot resume"
            return response
        self._resume_mission()
        response.success = True
        response.message = "Mission resumed"
        return response

    def _on_finish_mapping(self, _request, response: Trigger.Response) -> Trigger.Response:
        valid_state = self.state in (self.STATE_AUTO_MAP_V1, self.STATE_AUTO_EXPLORE)
        valid_paused_state = self.state == self.STATE_PAUSED and self.resume_state in (
            self.STATE_AUTO_MAP_V1,
            self.STATE_AUTO_EXPLORE,
        )
        if valid_state or valid_paused_state:
            self.finish_mapping_requested = True
            response.success = True
            response.message = "Mapping finish requested"
            return response
        response.success = False
        response.message = (
            "finish_mapping is valid only during AUTO_MAP_V1/AUTO_EXPLORE "
            f"(state={self.state})"
        )
        return response

    def _on_manual_override(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        if new_state == self.manual_override_active:
            return

        self.manual_override_active = new_state
        if self.manual_override_active:
            if self.state not in (
                self.STATE_IDLE,
                self.STATE_LOCALIZE_READY,
                self.STATE_ERROR,
                self.STATE_PAUSED,
            ):
                self._pause_mission("manual_override")
            self.get_logger().info("manual_override=true -> mission paused/canceled")
            return

        # manual override released
        if self.state == self.STATE_PAUSED and self.pause_reason == "manual_override":
            self.get_logger().info("manual_override=false -> auto resume")
            self._resume_mission()

    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_odom_xy is None:
            self.last_odom_xy = (x, y)
            return

        if self.state == self.STATE_AUTO_MAP_V1:
            dx = x - self.last_odom_xy[0]
            dy = y - self.last_odom_xy[1]
            self.mapping_distance_m += math.hypot(dx, dy)
        self.last_odom_xy = (x, y)

    def _on_frontier_done(self, msg: Bool) -> None:
        self.frontier_done = bool(msg.data)

    def _set_state(self, new_state: str) -> None:
        if new_state == self.state:
            return
        self.state = new_state
        self.state_entered_at = time.monotonic()
        self._publish_state()
        self.get_logger().info(f"State -> {self.state}")

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _start_new_mission(self) -> None:
        self._cancel_active_action("new_mission")
        self._stop_export_process()
        self._request_frontier_stop()

        self.home_pose = None
        self.map_name_current = self._resolve_map_name()
        self.finish_mapping_requested = False
        self.mapping_started_at = None
        self.mapping_distance_m = 0.0
        self.map_loop_count = 0
        self.map_failures = 0
        self.next_map_dispatch_at = 0.0
        self.return_home_attempts = 0
        self.last_odom_xy = None
        self.explore_started_at = None
        self.frontier_started = False
        self.frontier_done = False

        self.save_stage = "idle"
        self.write_future = None
        self.pbstream_path = None
        self.map_filestem = None

        self.resume_state = self.STATE_IDLE
        self.pause_reason = ""

        self._set_state(self.STATE_BOOT)

    def _resolve_map_name(self) -> str:
        if self.map_name_fixed:
            return self._resolve_unique_map_name(self.map_name_fixed)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return self._resolve_unique_map_name(f"{self.map_name_prefix}_{stamp}")

    def _map_artifacts_exist(self, map_name: str) -> bool:
        base = self.map_output_dir / map_name
        return any(
            [
                (base.with_suffix(".pbstream")).exists(),
                (base.with_suffix(".yaml")).exists(),
                (base.with_suffix(".pgm")).exists(),
            ]
        )

    def _resolve_unique_map_name(self, map_name: str) -> str:
        candidate = map_name
        if not self._map_artifacts_exist(candidate):
            return candidate

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        candidate = f"{map_name}_{stamp}"
        if not self._map_artifacts_exist(candidate):
            self.get_logger().warning(
                f"map_name '{map_name}' already exists, using '{candidate}'"
            )
            return candidate

        index = 1
        while True:
            candidate_with_index = f"{candidate}_{index}"
            if not self._map_artifacts_exist(candidate_with_index):
                self.get_logger().warning(
                    f"map_name '{map_name}' already exists, using '{candidate_with_index}'"
                )
                return candidate_with_index
            index += 1

    def _pause_mission(self, reason: str) -> None:
        if self.state == self.STATE_PAUSED:
            return
        self.resume_state = self.state
        self.pause_reason = reason
        self._cancel_active_action(f"pause:{reason}")
        if self.resume_state == self.STATE_AUTO_EXPLORE:
            self._request_frontier_stop()
            self.frontier_started = False
        self._set_state(self.STATE_PAUSED)

    def _resume_mission(self) -> None:
        if self.state != self.STATE_PAUSED:
            return
        target = self.resume_state or self.STATE_BOOT
        self.pause_reason = ""
        self.resume_state = self.STATE_IDLE
        if target == self.STATE_AUTO_EXPLORE:
            self.frontier_started = False
            self.frontier_done = False
        self._set_state(target)

    def _tick(self) -> None:
        if self.state == self.STATE_IDLE:
            return
        if self.state == self.STATE_PAUSED:
            return
        if self.state == self.STATE_ERROR:
            return
        if self.state == self.STATE_LOCALIZE_READY:
            return

        if self.state == self.STATE_BOOT:
            self._tick_boot()
            return

        if self.state == self.STATE_AUTO_MAP_V1:
            self._tick_auto_map_v1()
            return

        if self.state == self.STATE_AUTO_EXPLORE:
            self._tick_auto_explore()
            return

        if self.state == self.STATE_RETURN_HOME:
            self._tick_return_home()
            return

        if self.state == self.STATE_SAVE_EXPORT:
            self._tick_save_export()
            return

    def _tick_boot(self) -> None:
        elapsed = time.monotonic() - self.state_entered_at
        if elapsed < self.boot_capture_delay_sec:
            return

        pose = self._lookup_robot_pose_in_map()
        if pose is None:
            now = time.monotonic()
            if now - self.last_tf_warn_at > 2.0:
                self.get_logger().warning(
                    f"Waiting TF {self.map_frame}->{self.base_frame} to capture home pose..."
                )
                self.last_tf_warn_at = now
            return

        self.home_pose = pose
        self.mapping_started_at = None
        self.mapping_distance_m = 0.0
        self.map_loop_count = 0
        self.map_failures = 0
        self.finish_mapping_requested = False
        self.next_map_dispatch_at = time.monotonic()
        self.get_logger().info(
            f"Home pose captured: x={pose.x:.3f}, y={pose.y:.3f}, yaw={pose.yaw:.3f}"
        )
        if self.mapping_mode == "frontier":
            self.explore_started_at = time.monotonic()
            self.frontier_started = False
            self.frontier_done = False
            self._set_state(self.STATE_AUTO_EXPLORE)
        else:
            self._set_state(self.STATE_AUTO_MAP_V1)

    def _tick_auto_map_v1(self) -> None:
        if self.home_pose is None:
            self._set_error("home_pose missing in AUTO_MAP_V1")
            return

        now = time.monotonic()
        if self.mapping_started_at is None:
            self.mapping_started_at = now

        elapsed = now - self.mapping_started_at
        time_reached = self.mapping_timeout_sec > 0.0 and elapsed >= self.mapping_timeout_sec
        distance_reached = (
            self.mapping_max_distance_m > 0.0
            and self.mapping_distance_m >= self.mapping_max_distance_m
        )

        if self.finish_mapping_requested or time_reached or distance_reached:
            if self.action_inflight:
                self._cancel_active_action("mapping finish condition reached")
                return
            reason = []
            if self.finish_mapping_requested:
                reason.append("finish_mapping service")
            if time_reached:
                reason.append("time threshold")
            if distance_reached:
                reason.append("distance threshold")
            self.get_logger().info(
                "AUTO_MAP_V1 completed by " + (", ".join(reason) if reason else "condition")
            )
            self.return_home_attempts = 0
            self._set_state(self.STATE_RETURN_HOME)
            return

        if self.action_inflight:
            return

        if self.action_result is not None:
            action_name, success, status_code, message = self.action_result
            self.action_result = None
            if action_name != "auto_map_waypoints":
                self.get_logger().warning(
                    f"Unexpected action result in AUTO_MAP_V1: {action_name}"
                )
            if success:
                self.map_loop_count += 1
                self.map_failures = 0
                self.next_map_dispatch_at = now + self.mapping_loop_pause_sec
                self.get_logger().info(
                    f"AUTO_MAP_V1 loop {self.map_loop_count} done "
                    f"(elapsed={elapsed:.1f}s, distance={self.mapping_distance_m:.2f}m)"
                )
            else:
                self.map_failures += 1
                self.next_map_dispatch_at = now + self.mapping_retry_wait_sec
                self.get_logger().warning(
                    f"AUTO_MAP_V1 loop failed status={status_code} msg='{message}' "
                    f"(failure {self.map_failures}/{self.mapping_max_failures})"
                )
                if self.map_failures >= self.mapping_max_failures:
                    self._set_error("AUTO_MAP_V1 exceeded max failures")
                    return

        if now < self.next_map_dispatch_at:
            return

        poses = self._build_auto_map_waypoints()
        if not poses:
            self._set_error("AUTO_MAP_V1 has empty waypoint list")
            return
        if not self._dispatch_follow_waypoints(poses, "auto_map_waypoints"):
            self.next_map_dispatch_at = now + 0.5

    def _tick_auto_explore(self) -> None:
        if self.home_pose is None:
            self._set_error("home_pose missing in AUTO_EXPLORE")
            return

        if self.explore_started_at is None:
            self.explore_started_at = time.monotonic()

        if self.finish_mapping_requested:
            self._request_frontier_stop()
            self.return_home_attempts = 0
            self.get_logger().info("AUTO_EXPLORE completed by finish_mapping service")
            self._set_state(self.STATE_RETURN_HOME)
            return

        elapsed = time.monotonic() - self.explore_started_at
        if self.explore_timeout_sec > 0.0 and elapsed >= self.explore_timeout_sec:
            self._request_frontier_stop()
            self.return_home_attempts = 0
            self.get_logger().warning("AUTO_EXPLORE reached explore_timeout_sec")
            self._set_state(self.STATE_RETURN_HOME)
            return

        if not self.frontier_started:
            if self._request_frontier_start():
                self.frontier_started = True
                self.frontier_done = False
            return

        if self.frontier_done:
            self._request_frontier_stop()
            self.return_home_attempts = 0
            self.get_logger().info("AUTO_EXPLORE finished by frontier explorer done signal")
            self._set_state(self.STATE_RETURN_HOME)
            return

    def _tick_return_home(self) -> None:
        if self.home_pose is None:
            self._set_error("home_pose missing in RETURN_HOME")
            return

        if self.action_inflight:
            return

        if self.action_result is not None:
            action_name, success, status_code, message = self.action_result
            self.action_result = None
            if action_name != "return_home":
                self.get_logger().warning(
                    f"Unexpected action result in RETURN_HOME: {action_name}"
                )
            if success:
                self.get_logger().info("RETURN_HOME succeeded")
                self.save_stage = "idle"
                self._set_state(self.STATE_SAVE_EXPORT)
                return

            self.return_home_attempts += 1
            if self.return_home_attempts > self.home_retry_limit:
                self._set_error(
                    f"RETURN_HOME failed status={status_code} msg='{message}' after "
                    f"{self.return_home_attempts} attempts"
                )
                return
            self.get_logger().warning(
                f"RETURN_HOME retry {self.return_home_attempts}/{self.home_retry_limit} "
                f"(status={status_code}, msg='{message}')"
            )

        pose = self._pose2d_to_stamped(self.home_pose)
        self._dispatch_navigate_to_pose(pose, "return_home")

    def _tick_save_export(self) -> None:
        if self.save_stage == "idle":
            started = self._start_write_state()
            if started:
                self.save_stage = "writing"
            return

        if self.save_stage == "writing":
            if self.write_future is None:
                self._set_error("SAVE_EXPORT internal error: write_future is None")
                return
            if not self.write_future.done():
                return
            try:
                response = self.write_future.result()
            except Exception as exc:
                self._set_error(f"/write_state call failed: {exc}")
                return

            status_code = int(response.status.code)
            status_msg = str(response.status.message)
            if status_code != 0:
                self._set_error(
                    f"/write_state returned status={status_code} msg='{status_msg}'"
                )
                return

            self.get_logger().info(f"SAVE_EXPORT pbstream saved: {self.pbstream_path}")
            if self.export_map:
                if not self._start_export_map_process():
                    self._set_error("Failed to start map export process")
                    return
                self.save_stage = "exporting"
                return

            self.save_stage = "done"

        if self.save_stage == "exporting":
            if self.export_process is None:
                self._set_error("SAVE_EXPORT internal error: export_process is None")
                return
            code = self.export_process.poll()
            if code is None:
                return
            self.export_process = None
            if code != 0:
                self._set_error(f"map export failed with exit code {code}")
                return
            self.get_logger().info(f"SAVE_EXPORT map export finished: {self.map_filestem}")
            self.save_stage = "done"

        if self.save_stage == "done":
            self.get_logger().info("Mission reached LOCALIZE_READY")
            self._set_state(self.STATE_LOCALIZE_READY)

    def _lookup_robot_pose_in_map(self) -> Pose2D | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except Exception:
            return None

        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        q = tf_msg.transform.rotation
        yaw = _quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        return Pose2D(tx, ty, yaw)

    def _build_auto_map_waypoints(self) -> List[PoseStamped]:
        if self.home_pose is None:
            return []
        home = self.home_pose
        cos_yaw = math.cos(home.yaw)
        sin_yaw = math.sin(home.yaw)
        poses: List[PoseStamped] = []
        for offset in self.auto_map_waypoint_offsets:
            rx = offset.x
            ry = offset.y
            ax = home.x + cos_yaw * rx - sin_yaw * ry
            ay = home.y + sin_yaw * rx + cos_yaw * ry
            ayaw = _norm_angle(home.yaw + offset.yaw)
            poses.append(self._pose2d_to_stamped(Pose2D(ax, ay, ayaw)))
        if self.append_home_waypoint:
            poses.append(self._pose2d_to_stamped(home))
        return poses

    def _pose2d_to_stamped(self, pose: Pose2D) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _dispatch_follow_waypoints(self, poses: Sequence[PoseStamped], label: str) -> bool:
        if not self.follow_waypoints_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            self._warn_action_server_once("follow_waypoints")
            return False
        goal = FollowWaypoints.Goal()
        goal.poses = list(poses)
        return self._dispatch_action(self.follow_waypoints_client, goal, label)

    def _dispatch_navigate_to_pose(self, pose: PoseStamped, label: str) -> bool:
        if not self.navigate_to_pose_client.wait_for_server(
            timeout_sec=self.action_server_timeout_sec
        ):
            self._warn_action_server_once("navigate_to_pose")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return self._dispatch_action(self.navigate_to_pose_client, goal, label)

    def _warn_action_server_once(self, name: str) -> None:
        now = time.monotonic()
        if now - self.last_server_warn_at > 2.0:
            self.get_logger().warning(f"Waiting for action server: {name}")
            self.last_server_warn_at = now

    def _call_trigger_client(self, client, name: str, timeout_sec: float = 0.5) -> bool:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            now = time.monotonic()
            if now - self.last_server_warn_at > 2.0:
                self.get_logger().warning(f"Waiting service: {name}")
                self.last_server_warn_at = now
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            return False
        if not response.success:
            self.get_logger().warning(f"Service {name} failed: {response.message}")
            return False
        return True

    def _request_frontier_start(self) -> bool:
        if self.mapping_mode != "frontier":
            return False
        return self._call_trigger_client(self.frontier_start_client, self.frontier_start_service)

    def _request_frontier_stop(self) -> bool:
        if self.mapping_mode != "frontier":
            return False
        # Stop is best-effort during transitions/shutdown.
        try:
            return self._call_trigger_client(
                self.frontier_stop_client, self.frontier_stop_service, timeout_sec=0.3
            )
        except Exception:
            return False

    def _dispatch_action(self, client: ActionClient, goal_msg, label: str) -> bool:
        if self.action_inflight:
            return False
        self.action_token += 1
        token = self.action_token
        self.action_inflight = True
        self.action_result = None
        self.active_goal_handle = None
        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future, action_label=label, action_token=token: self._on_goal_response(
                future, action_label, action_token
            )
        )
        return True

    def _on_goal_response(self, future, label: str, token: int) -> None:
        if token != self.action_token:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.action_inflight = False
            self.action_result = (label, False, -1, f"goal send error: {exc}")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.action_inflight = False
            self.action_result = (label, False, -1, "goal rejected")
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, action_label=label, action_token=token: self._on_action_result(
                result, action_label, action_token
            )
        )

    def _on_action_result(self, future, label: str, token: int) -> None:
        if token != self.action_token:
            return
        self.action_inflight = False
        self.active_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:
            self.action_result = (label, False, -1, f"result error: {exc}")
            return

        if wrapped is None:
            self.action_result = (label, False, -1, "empty action result")
            return

        status = int(wrapped.status)
        success = status == GoalStatus.STATUS_SUCCEEDED
        detail = self._summarize_action_result(wrapped.result)
        self.action_result = (label, success, status, detail)

    def _cancel_active_action(self, reason: str) -> None:
        if not self.action_inflight and self.active_goal_handle is None:
            return

        self.action_token += 1
        self.action_inflight = False
        self.action_result = None

        goal_handle = self.active_goal_handle
        self.active_goal_handle = None
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.get_logger().info(f"Canceled active action ({reason})")

    def _start_write_state(self) -> bool:
        if not self.write_state_client.wait_for_service(timeout_sec=0.3):
            now = time.monotonic()
            if now - self.last_server_warn_at > 2.0:
                self.get_logger().warning(f"Waiting service: {self.write_state_service}")
                self.last_server_warn_at = now
            return False

        self.map_output_dir.mkdir(parents=True, exist_ok=True)
        self.pbstream_path = self.map_output_dir / f"{self.map_name_current}.pbstream"
        self.map_filestem = self.map_output_dir / self.map_name_current

        request = WriteState.Request()
        request.filename = str(self.pbstream_path)
        request.include_unfinished_submaps = self.include_unfinished_submaps
        self.write_future = self.write_state_client.call_async(request)
        return True

    def _start_export_map_process(self) -> bool:
        if self.pbstream_path is None or self.map_filestem is None:
            return False
        command = [
            "ros2",
            "run",
            "cartographer_ros",
            "cartographer_pbstream_to_ros_map",
            "-pbstream_filename",
            str(self.pbstream_path),
            "-map_filestem",
            str(self.map_filestem),
            "-resolution",
            str(self.export_resolution),
        ]
        try:
            self.export_process = subprocess.Popen(command)
        except Exception as exc:
            self.get_logger().error(f"Failed to start export process: {exc}")
            self.export_process = None
            return False
        return True

    def _stop_export_process(self) -> None:
        if self.export_process is None:
            return
        if self.export_process.poll() is None:
            try:
                self.export_process.terminate()
            except Exception:
                pass
        self.export_process = None

    def _set_error(self, message: str) -> None:
        self.get_logger().error(message)
        self._cancel_active_action("error")
        self._request_frontier_stop()
        self._stop_export_process()
        self._set_state(self.STATE_ERROR)

    def destroy_node(self) -> bool:
        self._cancel_active_action("shutdown")
        self._request_frontier_stop()
        self._stop_export_process()
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MissionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy_ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
