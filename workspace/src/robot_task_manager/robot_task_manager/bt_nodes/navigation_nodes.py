import py_trees
import rclpy
import threading
import time
import math

from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import py_trees

class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    BT Leaf Node that sends a NavigateToPose goal to the nav2 action server.
    """
    def __init__(self, goal_key: str, yaw: float = 0.0, frame_id: str = "map"):
        super().__init__(f"NavigateToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()
        goal = getattr(self.bb, self.goal_key, None)
        
        self.x = goal[0]
        self.y = goal[1]
        self.yaw = yaw
        self.frame_id = frame_id
        self.action_name = "navigate_to_pose"

        self.node = None
        self.client = None
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        self.start_time = None
        self.timeout_sec = 30.0

    def setup(self, **kwargs):
        if not rclpy.ok():
            return False
        node_name = f"bt_nav_goal_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = ActionClient(self.node, NavigateToPose, self.action_name)
        return True

    def initialise(self):
        if not self.client.wait_for_server(timeout_sec=self.timeout_sec):
            self.feedback_message = f"Action server {self.action_name} not available"
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
        self.goal_handle = None
        self.result_future = None

    def update(self):
        if self.send_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.send_future.done():
                self.goal_handle = self.send_future.result()
                self.send_future = None
                if self.goal_handle is None or not self.goal_handle.accepted:
                    self.feedback_message = "Goal rejected"
                    return py_trees.common.Status.FAILURE
                self.result_future = self.goal_handle.get_result_async()
                self.feedback_message = "Goal accepted, waiting for result..."
                return py_trees.common.Status.RUNNING
            else:
                return py_trees.common.Status.RUNNING

        if self.result_future is not None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.result_future.done():
                result = self.result_future.result()
                if result is None:
                    self.feedback_message = "Result missing or timeout"
                    return py_trees.common.Status.FAILURE
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.feedback_message = "Goal succeeded"
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = f"Goal failed with status {result.status}"
                    return py_trees.common.Status.FAILURE
            else:
                # optional timeout check
                if time.monotonic() - self.start_time > self.timeout_sec:
                    self.feedback_message = "Goal timed out"
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        self.client = None
        self.send_future = None
        self.result_future = None
        self.goal_handle = None


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