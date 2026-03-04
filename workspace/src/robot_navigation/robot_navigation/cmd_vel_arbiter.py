#!/usr/bin/env python3
"""
Arbitrate manual and autonomous cmd_vel streams into a single output topic.

Priority:
1. /manual_override == true: only manual command is allowed.
2. /manual_override == false: manual still has priority when active.
3. If neither source is fresh, publish zero velocity.
"""

from __future__ import annotations

import time
from typing import Iterable, List, Sequence

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.utilities import ok as rclpy_ok
from std_msgs.msg import Bool


def _copy_twist(msg: Twist) -> Twist:
    copied = Twist()
    copied.linear.x = msg.linear.x
    copied.linear.y = msg.linear.y
    copied.linear.z = msg.linear.z
    copied.angular.x = msg.angular.x
    copied.angular.y = msg.angular.y
    copied.angular.z = msg.angular.z
    return copied


class CmdVelArbiter(Node):
    """Manual-first cmd_vel arbiter with explicit manual override support."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_arbiter")

        self.declare_parameter("manual_cmd_topic", "/cmd_vel_manual")
        self.declare_parameter(
            "auto_cmd_topics",
            ["/cmd_vel_auto", "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel"],
        )
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("manual_override_topic", "/manual_override")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("manual_cmd_timeout", 0.35)
        self.declare_parameter("auto_cmd_timeout", 0.35)
        self.declare_parameter("stop_on_source_switch", True)

        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value).strip()
        self.output_cmd_topic = str(self.get_parameter("output_cmd_topic").value).strip()
        self.manual_override_topic = str(self.get_parameter("manual_override_topic").value).strip()
        auto_topics_param = self.get_parameter("auto_cmd_topics").value
        auto_topics = self._resolve_topics(auto_topics_param)

        # Avoid self-loop if output topic is accidentally added as an auto source.
        self.auto_cmd_topics = [topic for topic in auto_topics if topic != self.output_cmd_topic]
        if len(self.auto_cmd_topics) < len(auto_topics):
            self.get_logger().warning(
                f"Removed output topic '{self.output_cmd_topic}' from auto_cmd_topics to avoid loop."
            )
        if not self.auto_cmd_topics:
            raise ValueError("auto_cmd_topics cannot be empty after filtering output topic.")

        publish_hz = float(self.get_parameter("publish_rate").value)
        self.publish_period = 1.0 / max(1e-3, publish_hz)
        self.manual_cmd_timeout = max(
            0.0, float(self.get_parameter("manual_cmd_timeout").value)
        )
        self.auto_cmd_timeout = max(0.0, float(self.get_parameter("auto_cmd_timeout").value))
        self.stop_on_source_switch = bool(self.get_parameter("stop_on_source_switch").value)

        self.manual_cmd = Twist()
        self.auto_cmd = Twist()
        self.last_manual_cmd_at: float | None = None
        self.last_auto_cmd_at: float | None = None
        self.last_auto_topic = ""
        self.manual_override = False
        self.active_source = "stop"

        qos = QoSProfile(depth=10)
        self.output_pub = self.create_publisher(Twist, self.output_cmd_topic, qos)
        self.manual_sub = self.create_subscription(
            Twist, self.manual_cmd_topic, self._on_manual_cmd, qos
        )
        self.auto_subs = [
            self.create_subscription(
                Twist,
                topic,
                self._make_auto_cb(topic),
                qos,
            )
            for topic in self.auto_cmd_topics
        ]
        self.manual_override_sub = self.create_subscription(
            Bool,
            self.manual_override_topic,
            self._on_manual_override,
            qos,
        )
        self.publish_timer = self.create_timer(self.publish_period, self._publish_cycle)

        self.get_logger().info(
            "cmd_vel_arbiter ready "
            f"(manual={self.manual_cmd_topic}, auto={self.auto_cmd_topics}, "
            f"override={self.manual_override_topic}, output={self.output_cmd_topic}, "
            f"manual_timeout={self.manual_cmd_timeout:.2f}s, "
            f"auto_timeout={self.auto_cmd_timeout:.2f}s)"
        )

    def _resolve_topics(self, values: Sequence[str] | str) -> List[str]:
        topics: List[str] = []
        if isinstance(values, (str, bytes)):
            raw = str(values).strip()
            if raw.startswith("[") and raw.endswith("]"):
                raw = raw[1:-1]
                values = [piece.strip().strip("\"'") for piece in raw.split(",")]
            else:
                values = [raw]
        if isinstance(values, Iterable):
            for item in values:
                topic = str(item).strip()
                if topic and topic not in topics:
                    topics.append(topic)
        return topics

    def _make_auto_cb(self, topic: str):
        def _cb(msg: Twist) -> None:
            self.auto_cmd = _copy_twist(msg)
            self.last_auto_cmd_at = time.monotonic()
            self.last_auto_topic = topic

        return _cb

    def _on_manual_cmd(self, msg: Twist) -> None:
        self.manual_cmd = _copy_twist(msg)
        self.last_manual_cmd_at = time.monotonic()

    def _on_manual_override(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        if new_state == self.manual_override:
            return
        self.manual_override = new_state

        if self.manual_override:
            self.output_pub.publish(Twist())
            self.get_logger().info("manual_override=true -> forced manual mode.")
        else:
            self.get_logger().info("manual_override=false -> auto mode can resume.")

    def _is_fresh(self, stamp: float | None, timeout: float) -> bool:
        if stamp is None:
            return False
        return (time.monotonic() - stamp) <= timeout

    def _select_source(self) -> tuple[str, Twist]:
        manual_fresh = self._is_fresh(self.last_manual_cmd_at, self.manual_cmd_timeout)
        auto_fresh = self._is_fresh(self.last_auto_cmd_at, self.auto_cmd_timeout)

        if self.manual_override:
            if manual_fresh:
                return "manual", self.manual_cmd
            return "stop", Twist()

        if manual_fresh:
            return "manual", self.manual_cmd
        if auto_fresh:
            return "auto", self.auto_cmd
        return "stop", Twist()

    def _publish_cycle(self) -> None:
        source, cmd = self._select_source()

        if self.stop_on_source_switch and source != self.active_source:
            self.output_pub.publish(Twist())
            if source == "auto":
                self.get_logger().info(
                    f"Control switched to auto ({self.last_auto_topic or 'unknown'})"
                )
            else:
                self.get_logger().info(f"Control switched to {source}")
            self.active_source = source
            return

        self.output_pub.publish(cmd)

    def destroy_node(self) -> bool:
        self.output_pub.publish(Twist())
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiter()
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
