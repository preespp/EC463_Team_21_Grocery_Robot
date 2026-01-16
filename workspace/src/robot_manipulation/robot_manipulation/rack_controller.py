#!/usr/bin/env python3
import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rack_interfaces.srv import MoveRack
from rack_interfaces.msg import RackCommand

from std_msgs.msg import String


class RackSystemMotorNode(Node):
    def __init__(self):
        super().__init__("rack_system_motor_node")

        # Publish commands to the motor node
        self.cmd_pub = self.create_publisher(RackCommand, "/rack/command", 10)

        # Subscribe to rack state
        self.state_sub = self.create_subscription(String, "/rack/state", self.on_state, 10)

        # Service for high-level requests
        self.srv = self.create_service(MoveRack, "/move_rack", self.on_move_rack)

        # Internal state tracking
        self.lock = threading.Lock()
        self.last_state: dict = {
            "rack_id": "",
            "state": "IDLE",
            "current_angle_deg": 0.0,
            "detail": "",
        }
        self.state_changed = threading.Event()

        self.get_logger().info("RackSystemMotorNode ready: service /move_rack, pub /rack/command, sub /rack/state")

    def on_state(self, msg: String):

        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"rack_id": "", "state": msg.data, "current_angle_deg": 0.0, "detail": ""}

        with self.lock:
            self.last_state.update({
                "rack_id": data.get("rack_id", ""),
                "state": data.get("state", "UNKNOWN"),
                "current_angle_deg": float(data.get("current_angle_deg", 0.0)),
                "detail": data.get("detail", ""),
            })
            self.state_changed.set()

    def on_move_rack(self, req: MoveRack.Request, resp: MoveRack.Response):
        # Basic validation
        if req.speed_deg_s <= 0.0:
            resp.success = False
            resp.message = "speed_deg_s must be > 0"
            return resp

        rack_id = req.rack_id if req.rack_id else "rack_default"

        # Publish command to motor node
        cmd = RackCommand()
        cmd.rack_id = rack_id
        cmd.target_angle_deg = float(req.target_angle_deg)
        cmd.speed_deg_s = float(req.speed_deg_s)
        cmd.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)

        # Immediately publish a local state update too (optional)
        self.publish_local_state(rack_id, "COMMAND_SENT", detail=f"target={req.target_angle_deg}, speed={req.speed_deg_s}")

        if not req.blocking:
            resp.success = True
            resp.message = "Command published (non-blocking)"
            return resp

        # If blocking=True: wait for DONE/ERROR from motor node
        ok, msg = self.wait_until_done(rack_id, timeout_sec=20.0)
        resp.success = ok
        resp.message = msg
        return resp

    def Await_until_done(self, rack_id: str, timeout_sec: float) -> tuple[bool, str]:
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)

        # clear event before waiting
        self._state_changed.clear()

        while self.get_clock().now() < deadline:
            # Wait for any state update
            self._state_changed.wait(timeout=0.2)
            self._state_changed.clear()

            with self._lock:
                s_rack = self._last_state.get("rack_id", "")
                s_state = self._last_state.get("state", "UNKNOWN")
                s_detail = self._last_state.get("detail", "")

            # If motor node doesn’t set rack_id, we still allow it
            if s_rack and s_rack != rack_id:
                continue

            if s_state == "DONE":
                return True, "Move complete (DONE)"
            if s_state == "ERROR":
                return False, f"Move failed (ERROR): {s_detail}"

        return False, f"Timeout waiting for DONE/ERROR ({timeout_sec:.1f}s)"


def main():
    rclpy.init()
    node = RackSystemMotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
