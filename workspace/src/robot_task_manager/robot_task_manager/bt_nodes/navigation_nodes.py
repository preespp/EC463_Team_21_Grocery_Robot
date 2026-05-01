import py_trees
import rclpy
import threading
import time
import math
from typing import Optional

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus


def _quaternion_to_yaw(orientation) -> float:
    return math.atan2(
        2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y)),
        1.0 - 2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)),
    )


class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    BT Leaf Node that sends a NavigateToPose goal to the nav2 action server.
    """
    def __init__(
        self,
        goal_key: str,
        bb=None,
        yaw: float = 0.0,
        frame_id: str = "map",
        timeout_sec: Optional[float] = None,
    ):
        super().__init__(f"NavigateToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.x = 0.0
        self.y = 0.0
        self.yaw = yaw
        self.frame_id = frame_id
        self.action_name = "navigate_to_pose"
        self.initialise_error = False

        self.node = None
        self.client = None
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        self.cancel_future = None
        self.start_time = None
        self.timeout_sec = None if timeout_sec is None else float(timeout_sec)
        self.active_timeout_sec = None

    def _dbg(self, msg: str):
        print(f"[NavigateToGoalPose] {msg}", flush=True)

    def _resolve_timeout_sec(self) -> Optional[float]:
        bb_timeout = getattr(self.bb, "nav_timeout_sec", None)
        if bb_timeout is not None:
            try:
                bb_timeout = float(bb_timeout)
            except (TypeError, ValueError):
                bb_timeout = None
            else:
                if bb_timeout > 0.0:
                    return bb_timeout
                return None

        if self.timeout_sec is not None and self.timeout_sec > 0.0:
            return self.timeout_sec
        return None

    def _cancel_goal(self, reason: str):
        if self.goal_handle is None or self.cancel_future is not None:
            return
        self.feedback_message = reason
        self._dbg(reason)
        self.cancel_future = self.goal_handle.cancel_goal_async()

    def setup(self, **kwargs):
        if self.client is not None:
            return True
        if not rclpy.ok():
            self._dbg("setup failed: rclpy not ok")
            return False
        node_name = f"bt_nav_goal_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, NavigateToPose, self.action_name)
        self._dbg(f"setup ok: node={node_name}, action={self.action_name}, goal_key={self.goal_key}")
        return True

    def initialise(self):
        self.initialise_error = False
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        self.cancel_future = None
        self.start_time = None
        self.active_timeout_sec = self._resolve_timeout_sec()

        goal = getattr(self.bb, self.goal_key, None)
        self._dbg(f"initialise: bb.{self.goal_key} raw={goal}")
        if goal is None:
            self.feedback_message = f"Invalid goal in bb.{self.goal_key}: {goal}"
            self.initialise_error = True
            self._dbg(f"FAIL: {self.feedback_message}")
            return

        try:
            self.x, self.y, self.yaw = self._parse_goal(goal)
            self._dbg(f"parsed goal: x={self.x}, y={self.y}, yaw={self.yaw}")
        except (TypeError, ValueError):
            self.feedback_message = f"Non-numeric goal in bb.{self.goal_key}: {goal}"
            self.initialise_error = True
            self._dbg(f"FAIL: {self.feedback_message}")
            return

        wait_for_server_timeout = self.active_timeout_sec if self.active_timeout_sec is not None else 2.0
        wait_for_server_timeout = max(2.0, wait_for_server_timeout)
        if not self.client.wait_for_server(timeout_sec=wait_for_server_timeout):
            self.feedback_message = f"Action server {self.action_name} not available"
            self.initialise_error = True
            self._dbg(f"FAIL: {self.feedback_message}")
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        # convert yaw to quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        self.send_future = self.client.send_goal_async(goal_msg)
        self.start_time = time.monotonic()
        self.feedback_message = f"Goal sent to {self.action_name}"
        self._dbg(
            f"goal sent: frame={self.frame_id} x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}"
        )
        self.goal_handle = None
        self.result_future = None

    def _parse_goal(self, goal):
        if isinstance(goal, PoseStamped):
            return (
                float(goal.pose.position.x),
                float(goal.pose.position.y),
                _quaternion_to_yaw(goal.pose.orientation),
            )

        if isinstance(goal, dict):
            return (
                float(goal.get("x", 0.0)),
                float(goal.get("y", 0.0)),
                float(goal.get("yaw", self.yaw)),
            )

        if isinstance(goal, (list, tuple)) and len(goal) >= 2:
            yaw = float(goal[2]) if len(goal) >= 3 else float(self.yaw)
            return float(goal[0]), float(goal[1]), yaw

        raise TypeError(f"Unsupported goal type: {type(goal)}")

    def update(self):
        if self.initialise_error:
            self._dbg(f"update -> FAILURE (initialise_error): {self.feedback_message}")
            return py_trees.common.Status.FAILURE

        elapsed = None
        if self.start_time is not None:
            elapsed = time.monotonic() - self.start_time

        if self.send_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.send_future.done():
                self.goal_handle = self.send_future.result()
                self.send_future = None
                if self.goal_handle is None or not self.goal_handle.accepted:
                    self.feedback_message = "Goal rejected"
                    self._dbg("update -> FAILURE: goal rejected")
                    return py_trees.common.Status.FAILURE
                self.result_future = self.goal_handle.get_result_async()
                self.feedback_message = "Goal accepted, waiting for result..."
                self._dbg("goal accepted, waiting for result")
                return py_trees.common.Status.RUNNING
            else:
                if (
                    self.active_timeout_sec is not None
                    and elapsed is not None
                    and elapsed > self.active_timeout_sec
                ):
                    self.feedback_message = (
                        f"Goal send timed out after {elapsed:.1f}s before acceptance"
                    )
                    self._dbg(f"update -> FAILURE: {self.feedback_message}")
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING

        if self.result_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.result_future.done():
                result = self.result_future.result()
                if result is None:
                    self.feedback_message = "Result missing or timeout"
                    self._dbg("update -> FAILURE: result missing/timeout")
                    return py_trees.common.Status.FAILURE
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.feedback_message = "Goal succeeded"
                    self._dbg("update -> SUCCESS: goal succeeded")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = f"Goal failed with status {result.status}"
                    self._dbg(f"update -> FAILURE: {self.feedback_message}")
                    return py_trees.common.Status.FAILURE
            else:
                if (
                    self.active_timeout_sec is not None
                    and elapsed is not None
                    and elapsed > self.active_timeout_sec
                ):
                    self._cancel_goal(
                        f"Goal timed out after {elapsed:.1f}s; canceling active Nav2 goal"
                    )
                if self.cancel_future is not None:
                    if self.cancel_future.done():
                        self.feedback_message = "Waiting for canceled Nav2 goal result..."
                    else:
                        self.feedback_message = "Canceling timed-out Nav2 goal..."
                return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self.goal_handle is not None
            and self.result_future is not None
            and not self.result_future.done()
        ):
            self._cancel_goal("BT invalidated navigation goal; canceling active Nav2 goal")
        self.send_future = None
        self.result_future = None
        self.goal_handle = None
        self.cancel_future = None
        self.start_time = None
        self.active_timeout_sec = None


class MaybeNavigateToGoalPose(NavigateToGoalPose):
    """
    Navigate to bb.goal_key unless the blackboard requests navigation to be skipped.

    This is useful for bench tests where the arm logic should run without waiting on Nav2.
    """

    def __init__(
        self,
        goal_key: str,
        bb=None,
        yaw: float = 0.0,
        frame_id: str = "map",
        skip_flag_key: str = "skip_navigation",
    ):
        super().__init__(goal_key=goal_key, bb=bb, yaw=yaw, frame_id=frame_id)
        self.name = f"MaybeNavigateToGoalPose[{goal_key}]"
        self.skip_flag_key = skip_flag_key
        self.skip_navigation = False

    def initialise(self):
        self.skip_navigation = bool(getattr(self.bb, self.skip_flag_key, False))
        if self.skip_navigation:
            self.send_future = None
            self.result_future = None
            self.goal_handle = None
            self.feedback_message = (
                f"Skipping navigation because bb.{self.skip_flag_key} is true"
            )
            self._dbg(self.feedback_message)
            return
        super().initialise()

    def update(self):
        if self.skip_navigation:
            return py_trees.common.Status.SUCCESS
        return super().update()

    def terminate(self, new_status):
        self.skip_navigation = False
        super().terminate(new_status)


class ClearLocalCostmapAndSmartBackUp(py_trees.behaviour.Behaviour):
    """
    Clear the local costmap, then call Nav2's BackUp behavior action.

    The configured behavior plugin is robot_nav2_plugins/SmartBackUp, so this
    reuses the same free-space-aware retreat logic as the Nav2 recovery tree.
    The step is best-effort: return-home navigation still runs if backup fails.
    """

    def __init__(
        self,
        bb=None,
        clear_service_name: str = "local_costmap/clear_entirely_local_costmap",
        backup_action_name: str = "/backup",
        backup_distance_m: float = 0.12,
        backup_speed_mps: float = 0.18,
        clear_timeout_sec: float = 1.0,
        wait_after_clear_sec: float = 0.10,
        action_server_timeout_sec: float = 1.0,
        action_time_allowance_sec: float = 3.0,
    ):
        super().__init__("ClearLocalCostmapAndSmartBackUp")
        self.bb = bb if bb is not None else py_trees.blackboard.Blackboard()
        self.clear_service_name = clear_service_name
        self.backup_action_name = backup_action_name
        self.backup_distance_m = abs(float(backup_distance_m))
        self.backup_speed_mps = abs(float(backup_speed_mps))
        self.clear_timeout_sec = float(clear_timeout_sec)
        self.wait_after_clear_sec = max(0.0, float(wait_after_clear_sec))
        self.action_server_timeout_sec = float(action_server_timeout_sec)
        self.action_time_allowance_sec = float(action_time_allowance_sec)

        self.node = None
        self.clear_client = None
        self.backup_client = None
        self.clear_future = None
        self.clear_request_start_time = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None
        self.start_time = None
        self.initialise_error = False
        self.done = False

    def _dbg(self, msg: str):
        print(f"[ClearLocalCostmapAndSmartBackUp] {msg}", flush=True)

    def setup(self, **kwargs):
        if self.clear_client is not None and self.backup_client is not None:
            return True

        if not rclpy.ok():
            self._dbg("setup failed: rclpy not ok")
            return False

        node_name = f"bt_clear_and_smart_backup_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.clear_client = self.node.create_client(
            ClearEntireCostmap,
            self.clear_service_name,
        )
        self.backup_client = ActionClient(self.node, BackUp, self.backup_action_name)
        self._dbg(
            f"setup ok: clear_service={self.clear_service_name}, "
            f"backup_action={self.backup_action_name}"
        )
        return True

    def initialise(self):
        self.initialise_error = False
        self.clear_future = None
        self.clear_request_start_time = None
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None
        self.start_time = time.monotonic()
        self.done = False

        if self.clear_client is None or self.backup_client is None:
            self.initialise_error = True
            self.feedback_message = "clear client or backup action client not initialized"
            self._dbg(f"FAIL: {self.feedback_message}")
            return

        if bool(getattr(self.bb, "skip_navigation", False)):
            self.done = True
            self.feedback_message = "Skipping SmartBackUp because bb.skip_navigation is true"
            self._dbg(self.feedback_message)
            return

        if self.backup_distance_m <= 0.0 or self.backup_speed_mps <= 0.0:
            self.initialise_error = True
            self.feedback_message = "backup distance and speed must be > 0"
            self._dbg(f"FAIL: {self.feedback_message}")
            return

        if self.clear_client.wait_for_service(timeout_sec=self.clear_timeout_sec):
            self.clear_future = self.clear_client.call_async(ClearEntireCostmap.Request())
            self.clear_request_start_time = time.monotonic()
            self.feedback_message = "Clearing local costmap before SmartBackUp"
            self._dbg(self.feedback_message)
        else:
            self.feedback_message = (
                f"Local costmap clear service unavailable after "
                f"{self.clear_timeout_sec:.1f}s; calling SmartBackUp anyway"
            )
            self._dbg(self.feedback_message)
            self._send_backup_goal()

    def update(self):
        if self.initialise_error:
            return py_trees.common.Status.FAILURE

        if bool(getattr(self.bb, "skip_navigation", False)):
            return py_trees.common.Status.SUCCESS

        if self.done:
            return py_trees.common.Status.SUCCESS

        if self.clear_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.clear_future.done():
                if (
                    self.clear_request_start_time is not None
                    and time.monotonic() - self.clear_request_start_time > self.clear_timeout_sec
                ):
                    self._dbg(
                        f"local costmap clear timed out after "
                        f"{self.clear_timeout_sec:.1f}s; calling SmartBackUp anyway"
                    )
                    self.clear_future = None
                    self.clear_request_start_time = None
                    self._send_backup_goal()
                    return py_trees.common.Status.RUNNING
                return py_trees.common.Status.RUNNING
            try:
                self.clear_future.result()
            except Exception as exc:
                self._dbg(f"local costmap clear failed; calling SmartBackUp anyway: {exc}")
            self.clear_future = None
            self.clear_request_start_time = None
            if self.wait_after_clear_sec > 0.0:
                self.start_time = time.monotonic()
            else:
                self._send_backup_goal()
            return py_trees.common.Status.RUNNING

        if self.send_future is None and self.goal_handle is None and self.result_future is None:
            if (
                self.start_time is not None
                and time.monotonic() - self.start_time < self.wait_after_clear_sec
            ):
                return py_trees.common.Status.RUNNING
            self._send_backup_goal()
            return py_trees.common.Status.RUNNING

        if self.send_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.send_future.done():
                return py_trees.common.Status.RUNNING
            self.goal_handle = self.send_future.result()
            self.send_future = None
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.done = True
                self.feedback_message = "SmartBackUp goal rejected; continuing to Nav2 return home"
                self._dbg(self.feedback_message)
                return py_trees.common.Status.SUCCESS
            self.result_future = self.goal_handle.get_result_async()
            self.feedback_message = "SmartBackUp goal accepted"
            self._dbg(self.feedback_message)
            return py_trees.common.Status.RUNNING

        if self.result_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if not self.result_future.done():
                return py_trees.common.Status.RUNNING
            result = self.result_future.result()
            status = getattr(result, "status", None)
            self.result_future = None
            self.goal_handle = None
            self.done = True
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = (
                    f"SmartBackUp completed: distance={self.backup_distance_m:.2f}m "
                    f"speed={self.backup_speed_mps:.2f}m/s"
                )
            else:
                self.feedback_message = (
                    f"SmartBackUp finished with status {status}; continuing to Nav2 return home"
                )
            self._dbg(self.feedback_message)
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if (
            new_status == py_trees.common.Status.INVALID
            and self.goal_handle is not None
            and self.result_future is not None
            and not self.result_future.done()
            and self.cancel_future is None
        ):
            self.cancel_future = self.goal_handle.cancel_goal_async()
        self.clear_future = None
        self.clear_request_start_time = None
        self.send_future = None
        self.result_future = None
        self.goal_handle = None
        self.cancel_future = None
        self.start_time = None

    def shutdown(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
            self.clear_client = None
            self.backup_client = None

    def _send_backup_goal(self):
        if self.backup_client is None:
            self.done = True
            self.feedback_message = "SmartBackUp action client missing; continuing"
            self._dbg(self.feedback_message)
            return

        if not self.backup_client.wait_for_server(timeout_sec=self.action_server_timeout_sec):
            self.done = True
            self.feedback_message = (
                f"SmartBackUp action unavailable after "
                f"{self.action_server_timeout_sec:.1f}s; continuing to Nav2 return home"
            )
            self._dbg(self.feedback_message)
            return

        goal_msg = BackUp.Goal()
        goal_msg.target.x = self.backup_distance_m
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = self.backup_speed_mps
        goal_msg.time_allowance.sec = int(self.action_time_allowance_sec)
        goal_msg.time_allowance.nanosec = int(
            (self.action_time_allowance_sec - int(self.action_time_allowance_sec)) * 1e9
        )

        self.send_future = self.backup_client.send_goal_async(goal_msg)
        self.feedback_message = (
            f"Sent SmartBackUp: distance={self.backup_distance_m:.2f}m "
            f"speed={self.backup_speed_mps:.2f}m/s"
        )
        self._dbg(self.feedback_message)


# Backward-compatible alias for older trees/imports.
ClearLocalCostmapAndBackAway = ClearLocalCostmapAndSmartBackUp


############## Delete Later Only for Feature 1 Demo Need to Migrate to real auto nav script)
class MoveDistanceForCurrentItem(py_trees.behaviour.Behaviour):
    """
    Demo leaf: move a fixed distance in fixed time by publishing /cmd_vel.
    Publishes at 50Hz to match nav2_serial_bridge send cadence.
    """

    def __init__(self, bb):
        super().__init__("MoveDistanceForCurrentItem")
        self.bb = bb

        self.distance_m = 0.30
        self.duration_sec = 2.0
        self.publish_rate_hz = 50.0
        self.cmd_topic = "/cmd_vel"

        self.node = None
        self.pub = None
        self.start_time = None
        self.active = False
        self.cmd_linear_x = 0.0
        self.stop_event = threading.Event()
        self.publisher_thread = None

    def setup(self, **kwargs):
        if self.pub is not None:
            return True

        if not rclpy.ok():
            return False

        node_name = f"bt_move_distance_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.pub = self.node.create_publisher(Twist, self.cmd_topic, 10)

        self.stop_event.clear()
        self.publisher_thread = threading.Thread(
            target=self._publisher_loop,
            name=f"{node_name}_pub",
            daemon=True,
        )
        self.publisher_thread.start()
        return True

    def initialise(self):
        item = getattr(self.bb, "current_item", None)
        if item is None:
            self.start_time = None
            self.active = False
            self.feedback_message = "bb.current_item is None"
            return

        if self.duration_sec <= 1e-6:
            self.start_time = None
            self.active = False
            self.feedback_message = "duration_sec must be > 0"
            return

        self.start_time = time.monotonic()
        self.cmd_linear_x = self.distance_m / self.duration_sec
        self.active = True
        self.feedback_message = (
            f"item={getattr(item, 'name', 'unknown')} "
            f"distance={self.distance_m:.2f}m duration={self.duration_sec:.2f}s "
            f"speed={self.cmd_linear_x:.3f}m/s pub_rate={self.publish_rate_hz:.1f}Hz"
        )

    def update(self):
        if self.pub is None:
            self.feedback_message = "publisher not initialized (setup failed?)"
            return py_trees.common.Status.FAILURE

        if self.start_time is None:
            self.active = False
            self.publish_stop()
            return py_trees.common.Status.FAILURE

        elapsed = time.monotonic() - self.start_time
        if elapsed >= self.duration_sec:
            self.active = False
            self.publish_stop()
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.active = False
        self.publish_stop()

    def shutdown(self):
        self.active = False
        self.stop_event.set()
        if self.publisher_thread is not None:
            self.publisher_thread.join(timeout=0.2)
            self.publisher_thread = None

        self.publish_stop()
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
            self.pub = None

    def publish_linear_x(self, linear_x: float):
        if self.pub is None:
            return
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        self.pub.publish(cmd)

    def publish_stop(self):
        if self.pub is None:
            return
        self.pub.publish(Twist())

    def _publisher_loop(self):
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        while not self.stop_event.is_set():
            if self.active:
                self.publish_linear_x(self.cmd_linear_x)
            time.sleep(period)
################################################
