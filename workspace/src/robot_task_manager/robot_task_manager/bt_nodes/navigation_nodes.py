import py_trees
import rclpy
import threading
import time
from geometry_msgs.msg import Twist

class NavigateToGoalPose(py_trees.behaviour.Behaviour):
    """
    Reusable navigation leaf.
    Reads goal pose from bb.<goal_key>.

    Later you will implement Nav2 action call here.
    For now, it succeeds immediately if goal exists.
    """
    def __init__(self, goal_key: str):
        super().__init__(f"NavigateToGoalPose[{goal_key}]")
        self.goal_key = goal_key
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        goal = getattr(self.bb, self.goal_key, None)
        if goal is None:
            return py_trees.common.Status.FAILURE

        # TODO: replace with Nav2 action client call
        return py_trees.common.Status.SUCCESS


### Delete Later Only for Feature 1 Demo Need to Migrate to real auto nav script)
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
