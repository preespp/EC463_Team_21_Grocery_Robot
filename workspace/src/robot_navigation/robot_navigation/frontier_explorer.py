#!/usr/bin/env python3
"""
Frontier exploration node (P2).

Core behavior:
- Subscribe `/map`
- Extract frontiers: unknown cells adjacent to free cells
- Cluster frontiers (BFS)
- Generate candidate goals (nearest free neighbor around frontier centroid)
- Reachability check and path-cost estimation on free-space grid
- Score candidates: info_gain - path_cost - risk_cost
- Send NavigateToPose goals continuously
- Recovery helpers: timeout cancel + blacklist + cooldown
- Stop conditions:
  - no_frontier_rounds >= limit
  - new_area_ratio below threshold for a window
  - max explore time reached
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from rclpy.utilities import ok as rclpy_ok
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


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
class Candidate:
    mx: int
    my: int
    wx: float
    wy: float
    info_gain: float
    risk: float
    path_len: float
    score: float


class FrontierExplorer(Node):
    """Autonomous frontier goal generator using Nav2 NavigateToPose."""

    STATE_IDLE = "IDLE"
    STATE_RUNNING = "RUNNING"
    STATE_PAUSED = "PAUSED"
    STATE_DONE = "DONE"
    STATE_ERROR = "ERROR"

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self.declare_parameter("autostart", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("manual_override_topic", "/manual_override")
        self.declare_parameter("nav2_namespace", "")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("state_topic", "/frontier_explorer/state")
        self.declare_parameter("done_topic", "/frontier_explorer/done")
        self.declare_parameter("goal_topic", "/frontier_explorer/current_goal")
        self.declare_parameter("start_service", "/frontier_explorer/start")
        self.declare_parameter("stop_service", "/frontier_explorer/stop")
        self.declare_parameter("loop_rate_hz", 1.5)
        self.declare_parameter("min_frontier_cluster_size", 8)
        self.declare_parameter("goal_timeout_sec", 35.0)
        self.declare_parameter("blacklist_ttl_sec", 45.0)
        self.declare_parameter("blacklist_radius_m", 0.35)
        self.declare_parameter("no_frontier_rounds_limit", 5)
        self.declare_parameter("max_explore_time_sec", 20.0 * 60.0)
        self.declare_parameter("new_area_ratio_threshold", 0.01)
        self.declare_parameter("new_area_window_sec", 60.0)
        self.declare_parameter("score_w_gain", 1.0)
        self.declare_parameter("score_w_path", 0.8)
        self.declare_parameter("score_w_risk", 0.6)
        self.declare_parameter("max_candidate_path_len_m", 0.0)
        self.declare_parameter("risk_check_radius_cells", 2)
        self.declare_parameter("robot_cell_search_radius_cells", 3)
        self.declare_parameter("recovery_failures_threshold", 3)
        self.declare_parameter("recovery_wait_sec", 3.0)
        self.declare_parameter("action_server_timeout_sec", 0.3)

        self.autostart = bool(self.get_parameter("autostart").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.manual_override_topic = str(self.get_parameter("manual_override_topic").value)
        self.nav2_namespace = _normalize_namespace(
            str(self.get_parameter("nav2_namespace").value)
        )
        self.navigate_to_pose_action = self._resolve_nav2_action_name(
            str(self.get_parameter("navigate_to_pose_action").value)
        )
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.done_topic = str(self.get_parameter("done_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.start_service_name = str(self.get_parameter("start_service").value)
        self.stop_service_name = str(self.get_parameter("stop_service").value)

        self.loop_rate_hz = max(1e-3, float(self.get_parameter("loop_rate_hz").value))
        self.min_frontier_cluster_size = max(
            1, int(self.get_parameter("min_frontier_cluster_size").value)
        )
        self.goal_timeout_sec = max(1.0, float(self.get_parameter("goal_timeout_sec").value))
        self.blacklist_ttl_sec = max(1.0, float(self.get_parameter("blacklist_ttl_sec").value))
        self.blacklist_radius_m = max(0.05, float(self.get_parameter("blacklist_radius_m").value))
        self.no_frontier_rounds_limit = max(
            1, int(self.get_parameter("no_frontier_rounds_limit").value)
        )
        self.max_explore_time_sec = max(
            10.0, float(self.get_parameter("max_explore_time_sec").value)
        )
        self.new_area_ratio_threshold = max(
            0.0, float(self.get_parameter("new_area_ratio_threshold").value)
        )
        self.new_area_window_sec = max(
            5.0, float(self.get_parameter("new_area_window_sec").value)
        )
        self.score_w_gain = float(self.get_parameter("score_w_gain").value)
        self.score_w_path = float(self.get_parameter("score_w_path").value)
        self.score_w_risk = float(self.get_parameter("score_w_risk").value)
        self.max_candidate_path_len_m = max(
            0.0, float(self.get_parameter("max_candidate_path_len_m").value)
        )
        self.risk_check_radius_cells = max(
            1, int(self.get_parameter("risk_check_radius_cells").value)
        )
        self.robot_cell_search_radius_cells = max(
            0, int(self.get_parameter("robot_cell_search_radius_cells").value)
        )
        self.recovery_failures_threshold = max(
            1, int(self.get_parameter("recovery_failures_threshold").value)
        )
        self.recovery_wait_sec = max(0.0, float(self.get_parameter("recovery_wait_sec").value))
        self.action_server_timeout_sec = max(
            0.05, float(self.get_parameter("action_server_timeout_sec").value)
        )

        qos = QoSProfile(depth=10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, qos
        )
        self.manual_override_sub = self.create_subscription(
            Bool, self.manual_override_topic, self._on_manual_override, qos
        )

        self.state_pub = self.create_publisher(String, self.state_topic, qos)
        self.done_pub = self.create_publisher(Bool, self.done_topic, qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, qos)

        self.start_srv = self.create_service(Trigger, self.start_service_name, self._on_start)
        self.stop_srv = self.create_service(Trigger, self.stop_service_name, self._on_stop)

        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_to_pose_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.STATE_IDLE
        self.manual_override = False
        self.running = False
        self.done = False

        self.map_msg: OccupancyGrid | None = None
        self.map_data: List[int] = []
        self.map_w = 0
        self.map_h = 0
        self.map_res = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.started_at = 0.0
        self.no_frontier_rounds = 0
        self.consecutive_failures = 0
        self.recovery_until = 0.0
        self.low_gain_since: float | None = None
        self.free_area_history: Deque[Tuple[float, int]] = deque(maxlen=300)

        self.blacklist: List[Tuple[float, float, float]] = []
        self.last_goal_xy: Tuple[float, float] | None = None
        self.goal_sent_at = 0.0
        self.goal_inflight = False
        self.goal_handle = None
        self.goal_result: Tuple[bool, int, str] | None = None
        self.goal_token = 0

        self.last_server_warn_at = 0.0
        self.last_tf_warn_at = 0.0

        self.loop_timer = self.create_timer(1.0 / self.loop_rate_hz, self._tick)
        self._publish_state()
        if self.autostart:
            self._start_exploration()

        self.get_logger().info(
            "frontier_explorer ready "
            f"(autostart={self.autostart}, map_topic={self.map_topic}, "
            f"nav2_namespace={self.nav2_namespace or '/'}, "
            f"navigate_action={self.navigate_to_pose_action})"
        )

    def _resolve_nav2_action_name(self, action_name: str) -> str:
        cleaned = action_name.strip()
        if not cleaned:
            return cleaned
        if not self.nav2_namespace:
            return cleaned

        if cleaned.startswith("/") and cleaned != "/navigate_to_pose":
            return cleaned

        return f"{self.nav2_namespace}/{cleaned.lstrip('/')}"

    def _summarize_nav_result(self, result_msg: object) -> str:
        if result_msg is None:
            return ""

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

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_w = int(msg.info.width)
        self.map_h = int(msg.info.height)
        self.map_res = float(msg.info.resolution)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)
        self.map_data = list(msg.data)

    def _on_manual_override(self, msg: Bool) -> None:
        new_value = bool(msg.data)
        if new_value == self.manual_override:
            return
        self.manual_override = new_value
        if self.manual_override and self.running:
            self._cancel_goal("manual_override")
            self._set_state(self.STATE_PAUSED)
        elif (not self.manual_override) and self.running:
            self._set_state(self.STATE_RUNNING)

    def _on_start(self, _request, response: Trigger.Response) -> Trigger.Response:
        self._start_exploration()
        response.success = True
        response.message = "frontier exploration started"
        return response

    def _on_stop(self, _request, response: Trigger.Response) -> Trigger.Response:
        self._stop_exploration(mark_done=False, reason="stop_service")
        response.success = True
        response.message = "frontier exploration stopped"
        return response

    def _set_state(self, state: str) -> None:
        if self.state == state:
            return
        self.state = state
        self._publish_state()
        self.get_logger().info(f"State -> {self.state}")

    def _publish_state(self) -> None:
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        done_msg = Bool()
        done_msg.data = self.done
        self.done_pub.publish(done_msg)

    def _start_exploration(self) -> None:
        # Clear any stale goal from a previous run/restart before resetting state.
        self._cancel_goal("restart")
        self.running = True
        self.done = False
        self.started_at = time.monotonic()
        self.no_frontier_rounds = 0
        self.consecutive_failures = 0
        self.recovery_until = 0.0
        self.low_gain_since = None
        self.free_area_history.clear()
        self.blacklist.clear()
        self.last_goal_xy = None
        self.goal_result = None
        self._set_state(self.STATE_PAUSED if self.manual_override else self.STATE_RUNNING)
        self._publish_state()

    def _stop_exploration(self, mark_done: bool, reason: str) -> None:
        self.running = False
        self.done = bool(mark_done)
        self._cancel_goal(reason)
        if mark_done:
            self._set_state(self.STATE_DONE)
        else:
            self._set_state(self.STATE_IDLE)
        self._publish_state()

    def _finish_exploration(self, reason: str) -> None:
        self.get_logger().info(f"Exploration finished: {reason}")
        self._stop_exploration(mark_done=True, reason=reason)

    def _tick(self) -> None:
        if not self.running:
            return

        now = time.monotonic()
        self._prune_blacklist(now)

        if self.manual_override:
            return

        if self.map_msg is None or not self.map_data:
            return

        if self._check_done_conditions(now):
            return

        if self.goal_inflight:
            if (now - self.goal_sent_at) > self.goal_timeout_sec:
                self._cancel_goal("goal_timeout")
                self._register_goal_failure("goal_timeout")
            return

        if self.goal_result is not None:
            success, status_code, status_msg = self.goal_result
            self.goal_result = None
            if success:
                self.consecutive_failures = 0
                self.no_frontier_rounds = 0
            else:
                self._register_goal_failure(f"status={status_code} msg='{status_msg}'")

        if now < self.recovery_until:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            now_tf = time.monotonic()
            if now_tf - self.last_tf_warn_at > 2.0:
                self.get_logger().warning(
                    f"Waiting TF {self.global_frame}->{self.base_frame} for frontier scoring..."
                )
                self.last_tf_warn_at = now_tf
            return

        candidates = self._extract_scored_candidates(robot_pose[0], robot_pose[1])
        if not candidates:
            self.no_frontier_rounds += 1
            self.get_logger().info(
                f"No valid frontier candidate (round={self.no_frontier_rounds}/{self.no_frontier_rounds_limit})"
            )
            if self.no_frontier_rounds >= self.no_frontier_rounds_limit:
                self._finish_exploration("no_frontier_rounds_limit")
            return

        best = max(candidates, key=lambda c: c.score)
        self._dispatch_goal(best.wx, best.wy, target_yaw=robot_pose[2])

    def _check_done_conditions(self, now: float) -> bool:
        elapsed = now - self.started_at
        if elapsed >= self.max_explore_time_sec:
            self._finish_exploration("max_explore_time")
            return True

        free_count = self._count_free_cells()
        self.free_area_history.append((now, free_count))
        while self.free_area_history and (now - self.free_area_history[0][0]) > self.new_area_window_sec:
            self.free_area_history.popleft()

        if len(self.free_area_history) >= 2:
            old_time, old_free = self.free_area_history[0]
            dt = now - old_time
            if dt >= self.new_area_window_sec * 0.7:
                ratio = (free_count - old_free) / max(1.0, float(old_free))
                if ratio < self.new_area_ratio_threshold:
                    if self.low_gain_since is None:
                        self.low_gain_since = now
                    elif (now - self.low_gain_since) >= self.new_area_window_sec:
                        self._finish_exploration("low_new_area_ratio")
                        return True
                else:
                    self.low_gain_since = None
        return False

    def _count_free_cells(self) -> int:
        return sum(1 for value in self.map_data if value == 0)

    def _lookup_robot_pose(self) -> Tuple[float, float, float] | None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
            )
        except Exception:
            return None

        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        q = tf_msg.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (tx, ty, yaw)

    def _extract_scored_candidates(self, robot_x: float, robot_y: float) -> List[Candidate]:
        robot_cell = self._world_to_map(robot_x, robot_y)
        if robot_cell is None:
            return []
        robot_start_idx = self._resolve_robot_free_cell(robot_cell[0], robot_cell[1])
        if robot_start_idx is None:
            return []
        distance_steps = self._build_reachability_field(robot_start_idx)

        frontier_flags = [False] * (self.map_w * self.map_h)

        for idx, value in enumerate(self.map_data):
            if value != -1:
                continue
            mx = idx % self.map_w
            my = idx // self.map_w
            if self._has_free_neighbor(mx, my):
                frontier_flags[idx] = True

        visited = [False] * len(frontier_flags)
        candidates: List[Candidate] = []

        for idx, is_frontier in enumerate(frontier_flags):
            if (not is_frontier) or visited[idx]:
                continue
            cluster, free_neighbors = self._bfs_frontier_cluster(idx, frontier_flags, visited)
            if len(cluster) < self.min_frontier_cluster_size:
                continue
            if not free_neighbors:
                continue

            reachable_neighbors = [ci for ci in free_neighbors if distance_steps[ci] >= 0]
            if not reachable_neighbors:
                continue

            centroid_mx = sum((cell % self.map_w) for cell in cluster) / float(len(cluster))
            centroid_my = sum((cell // self.map_w) for cell in cluster) / float(len(cluster))
            candidate_idx = min(
                reachable_neighbors,
                key=lambda ci: (
                    (ci % self.map_w - centroid_mx) ** 2 + (ci // self.map_w - centroid_my) ** 2
                ),
            )
            cmx = candidate_idx % self.map_w
            cmy = candidate_idx // self.map_w
            cwx, cwy = self._map_to_world(cmx, cmy)

            if self._is_blacklisted(cwx, cwy):
                continue

            path_len = self._path_length_if_reachable(distance_steps, candidate_idx)
            if path_len is None:
                continue
            risk = self._estimate_risk(cmx, cmy)
            info_gain = float(len(cluster))
            score = (
                self.score_w_gain * info_gain
                - self.score_w_path * path_len
                - self.score_w_risk * risk
            )
            candidates.append(
                Candidate(
                    mx=cmx,
                    my=cmy,
                    wx=cwx,
                    wy=cwy,
                    info_gain=info_gain,
                    risk=risk,
                    path_len=path_len,
                    score=score,
                )
            )
        return candidates

    def _path_length_if_reachable(
        self, distance_steps: Sequence[int], candidate_idx: int
    ) -> float | None:
        path_steps = int(distance_steps[candidate_idx])
        if path_steps < 0:
            return None
        path_len = float(path_steps) * self.map_res
        if self.max_candidate_path_len_m > 0.0 and path_len > self.max_candidate_path_len_m:
            return None
        return path_len

    def _bfs_frontier_cluster(
        self, start_idx: int, frontier_flags: Sequence[bool], visited: List[bool]
    ) -> Tuple[List[int], List[int]]:
        queue = deque([start_idx])
        visited[start_idx] = True
        cluster: List[int] = []
        free_neighbors_set: Dict[int, None] = {}

        while queue:
            idx = queue.popleft()
            cluster.append(idx)
            mx = idx % self.map_w
            my = idx // self.map_w

            for nx, ny in self._neighbors4(mx, my):
                nidx = ny * self.map_w + nx
                value = self.map_data[nidx]
                if value == 0:
                    free_neighbors_set[nidx] = None
                if frontier_flags[nidx] and (not visited[nidx]):
                    visited[nidx] = True
                    queue.append(nidx)

        return cluster, list(free_neighbors_set.keys())

    def _neighbors4(self, x: int, y: int) -> List[Tuple[int, int]]:
        result: List[Tuple[int, int]] = []
        if x > 0:
            result.append((x - 1, y))
        if x + 1 < self.map_w:
            result.append((x + 1, y))
        if y > 0:
            result.append((x, y - 1))
        if y + 1 < self.map_h:
            result.append((x, y + 1))
        return result

    def _has_free_neighbor(self, x: int, y: int) -> bool:
        for nx, ny in self._neighbors4(x, y):
            nidx = ny * self.map_w + nx
            if self.map_data[nidx] == 0:
                return True
        return False

    def _estimate_risk(self, mx: int, my: int) -> float:
        r = self.risk_check_radius_cells
        occ = 0
        total = 0
        for yy in range(max(0, my - r), min(self.map_h - 1, my + r) + 1):
            for xx in range(max(0, mx - r), min(self.map_w - 1, mx + r) + 1):
                total += 1
                idx = yy * self.map_w + xx
                if self.map_data[idx] > 50:
                    occ += 1
        if total == 0:
            return 1.0
        return float(occ) / float(total)

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        wx = self.map_origin_x + (mx + 0.5) * self.map_res
        wy = self.map_origin_y + (my + 0.5) * self.map_res
        return (wx, wy)

    def _world_to_map(self, wx: float, wy: float) -> Tuple[int, int] | None:
        if self.map_w <= 0 or self.map_h <= 0 or self.map_res <= 0.0:
            return None
        mx = int(math.floor((wx - self.map_origin_x) / self.map_res))
        my = int(math.floor((wy - self.map_origin_y) / self.map_res))
        if mx < 0 or my < 0 or mx >= self.map_w or my >= self.map_h:
            return None
        return (mx, my)

    def _resolve_robot_free_cell(self, robot_mx: int, robot_my: int) -> int | None:
        if robot_mx < 0 or robot_my < 0 or robot_mx >= self.map_w or robot_my >= self.map_h:
            return None
        origin_idx = robot_my * self.map_w + robot_mx
        if self.map_data[origin_idx] == 0:
            return origin_idx

        radius = self.robot_cell_search_radius_cells
        best_idx: int | None = None
        best_dist2 = float("inf")
        for yy in range(max(0, robot_my - radius), min(self.map_h - 1, robot_my + radius) + 1):
            for xx in range(max(0, robot_mx - radius), min(self.map_w - 1, robot_mx + radius) + 1):
                idx = yy * self.map_w + xx
                if self.map_data[idx] != 0:
                    continue
                dist2 = float((xx - robot_mx) ** 2 + (yy - robot_my) ** 2)
                if dist2 < best_dist2:
                    best_dist2 = dist2
                    best_idx = idx
        return best_idx

    def _build_reachability_field(self, start_idx: int) -> List[int]:
        distances = [-1] * (self.map_w * self.map_h)
        queue = deque([start_idx])
        distances[start_idx] = 0

        while queue:
            idx = queue.popleft()
            mx = idx % self.map_w
            my = idx // self.map_w
            next_dist = distances[idx] + 1
            for nx, ny in self._neighbors4(mx, my):
                nidx = ny * self.map_w + nx
                if distances[nidx] >= 0:
                    continue
                if self.map_data[nidx] != 0:
                    continue
                distances[nidx] = next_dist
                queue.append(nidx)
        return distances

    def _is_blacklisted(self, wx: float, wy: float) -> bool:
        for bx, by, _expiry in self.blacklist:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius_m:
                return True
        return False

    def _prune_blacklist(self, now: float) -> None:
        self.blacklist = [(x, y, t) for (x, y, t) in self.blacklist if t > now]

    def _dispatch_goal(self, wx: float, wy: float, target_yaw: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=self.action_server_timeout_sec):
            now = time.monotonic()
            if now - self.last_server_warn_at > 2.0:
                self.get_logger().warning(
                    f"Waiting for action server: {self.navigate_to_pose_action}"
                )
                self.last_server_warn_at = now
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = wx
        goal_pose.pose.position.y = wy
        qx, qy, qz, qw = _yaw_to_quaternion(_norm_angle(target_yaw))
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.goal_token += 1
        token = self.goal_token
        self.goal_inflight = True
        self.goal_result = None
        self.last_goal_xy = (wx, wy)
        self.goal_sent_at = time.monotonic()

        self.goal_pub.publish(goal_pose)
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, action_token=token: self._on_goal_response(future, action_token)
        )

    def _on_goal_response(self, future, token: int) -> None:
        if token != self.goal_token:
            return
        try:
            handle = future.result()
        except Exception as exc:
            self.goal_inflight = False
            self.goal_result = (False, -1, f"send_goal error: {exc}")
            return

        if handle is None or not handle.accepted:
            self.goal_inflight = False
            self.goal_result = (False, -1, "goal rejected")
            return

        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda rf, action_token=token: self._on_goal_result(rf, action_token)
        )

    def _on_goal_result(self, future, token: int) -> None:
        if token != self.goal_token:
            return
        self.goal_inflight = False
        self.goal_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:
            self.goal_result = (False, -1, f"result error: {exc}")
            return

        if wrapped is None:
            self.goal_result = (False, -1, "empty result")
            return

        status = int(wrapped.status)
        detail = self._summarize_nav_result(wrapped.result)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_result = (True, status, detail)
        else:
            self.goal_result = (False, status, detail)

    def _cancel_goal(self, reason: str) -> None:
        had_active_goal = self.goal_inflight or self.goal_handle is not None
        if self.goal_inflight and self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_token += 1
        self.goal_inflight = False
        self.goal_handle = None
        self.goal_result = None
        if had_active_goal:
            self.get_logger().info(f"Goal canceled ({reason})")

    def _register_goal_failure(self, reason: str) -> None:
        self.consecutive_failures += 1
        if self.last_goal_xy is not None:
            bx, by = self.last_goal_xy
            self.blacklist.append((bx, by, time.monotonic() + self.blacklist_ttl_sec))
        self.goal_inflight = False
        self.goal_handle = None
        self.goal_result = None

        self.get_logger().warning(
            f"Frontier goal failed: {reason} "
            f"(consecutive_failures={self.consecutive_failures})"
        )
        if self.consecutive_failures >= self.recovery_failures_threshold:
            self.recovery_until = time.monotonic() + self.recovery_wait_sec
            self.get_logger().warning(
                f"Recovery cooldown for {self.recovery_wait_sec:.1f}s after repeated failures"
            )
            self.consecutive_failures = 0

    def destroy_node(self) -> bool:
        self._cancel_goal("shutdown")
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
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
