import math

import py_trees
import rclpy
from robot_interfaces.srv import ResolveSemanticTarget


def _quaternion_to_yaw(orientation) -> float:
    return math.atan2(
        2.0 * ((orientation.w * orientation.z) + (orientation.x * orientation.y)),
        1.0 - 2.0 * ((orientation.y * orientation.y) + (orientation.z * orientation.z)),
    )


class ResolveCurrentItemSemanticTarget(py_trees.behaviour.Behaviour):
    """
    Resolve the current order item into semantic navigation and service targets.

    Falls back to the legacy x/y/z fields already stored on the blackboard when the
    semantic map server is unavailable or has no matching product entry.
    """

    def __init__(
        self,
        bb,
        service_name: str = "/semantic_map/resolve_target",
        fallback_to_legacy: bool = True,
        service_timeout_sec: float = 1.5,
    ):
        super().__init__("ResolveCurrentItemSemanticTarget")
        self.bb = bb
        self.service_name = service_name
        self.fallback_to_legacy = fallback_to_legacy
        self.service_timeout_sec = service_timeout_sec

        self.node = None
        self.client = None
        self.future = None
        self.final_status = None

    def setup(self, **kwargs):
        if self.client is not None:
            return True

        if not rclpy.ok():
            return False

        node_name = f"bt_semantic_target_{id(self) & 0xFFFF:x}"
        self.node = rclpy.create_node(node_name)
        self.client = self.node.create_client(ResolveSemanticTarget, self.service_name)
        return True

    def initialise(self):
        self.future = None
        self.final_status = None

        item = getattr(self.bb, "current_item", None)
        if item is None:
            self.feedback_message = "bb.current_item is None"
            self.final_status = py_trees.common.Status.FAILURE
            return

        if self.client is None:
            self.feedback_message = "semantic client not initialized"
            self.final_status = self._legacy_or_failure(item, self.feedback_message)
            return

        if not self.client.wait_for_service(timeout_sec=self.service_timeout_sec):
            self.feedback_message = f"semantic service unavailable: {self.service_name}"
            self.final_status = self._legacy_or_failure(item, self.feedback_message)
            return

        request = ResolveSemanticTarget.Request()
        request.product_id = str(getattr(item, "product_id", ""))
        request.product_name = str(getattr(item, "name", ""))
        self.future = self.client.call_async(request)
        self.feedback_message = (
            f"Resolving semantic target for item={request.product_name or request.product_id}"
        )

    def update(self):
        if self.final_status is not None:
            return self.final_status

        if self.future is None:
            self.feedback_message = "semantic resolution did not start"
            return py_trees.common.Status.FAILURE

        rclpy.spin_once(self.node, timeout_sec=0.1)
        if not self.future.done():
            return py_trees.common.Status.RUNNING

        item = getattr(self.bb, "current_item", None)
        try:
            response = self.future.result()
        except Exception as exc:
            self.feedback_message = f"semantic resolution error: {exc}"
            self.final_status = self._legacy_or_failure(item, self.feedback_message)
            return self.final_status

        if response is None:
            self.feedback_message = "semantic resolution returned no response"
            self.final_status = self._legacy_or_failure(item, self.feedback_message)
            return self.final_status

        if not response.found:
            self.feedback_message = response.message or "semantic target not found"
            self.final_status = self._legacy_or_failure(item, self.feedback_message)
            return self.final_status

        self._apply_response(response)
        self.feedback_message = response.message or f"resolved {response.target_label}"
        self.final_status = py_trees.common.Status.SUCCESS
        return self.final_status

    def terminate(self, new_status):
        self.future = None
        self.final_status = None

    def _legacy_or_failure(self, item, reason: str):
        if not self.fallback_to_legacy or item is None:
            return py_trees.common.Status.FAILURE

        self.bb.slot_id = None
        self.bb.anchor_id = None
        self.bb.rack_id = None
        self.bb.semantic_id = None
        self.bb.semantic_target_label = None
        self.bb.nav_goal_source = "legacy"
        self.bb.shelf_pose = None
        self.bb.pose = None
        self.feedback_message = f"{reason}; falling back to legacy coordinates"
        return py_trees.common.Status.SUCCESS

    def _apply_response(self, response: ResolveSemanticTarget.Response):
        nav_pose = response.nav_pose
        service_pose = response.service_pose
        nav_yaw = _quaternion_to_yaw(nav_pose.pose.orientation)

        self.bb.nav_goal = (
            float(nav_pose.pose.position.x),
            float(nav_pose.pose.position.y),
            float(nav_yaw),
        )
        if response.rack_level > 0:
            self.bb.rack_goal = int(response.rack_level)
        self.bb.shelf_pose = service_pose
        self.bb.pose = service_pose
        self.bb.slot_id = response.slot_id or None
        self.bb.anchor_id = response.anchor_id or None
        self.bb.rack_id = response.rack_id or None
        self.bb.semantic_id = response.semantic_id or None
        self.bb.semantic_target_label = response.target_label or None
        self.bb.nav_goal_source = response.resolved_by or "semantic_map"
