#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LaserScanSanitizer(Node):
    """Republishes LaserScan messages with sane range bounds for Cartographer."""

    def __init__(self) -> None:
        super().__init__("laser_scan_sanitizer")
        self.declare_parameter("fallback_range_max", 10.0)
        self.declare_parameter("warn_period_sec", 5.0)

        self.fallback_range_max = float(self.get_parameter("fallback_range_max").value)
        self.warn_period_sec = max(0.0, float(self.get_parameter("warn_period_sec").value))
        self.last_warn_time = 0.0

        self.publisher = self.create_publisher(LaserScan, "scan_sanitized", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan,
            "scan",
            self._handle_scan,
            qos_profile_sensor_data,
        )

    def _handle_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time

        range_min = float(msg.range_min)
        if not math.isfinite(range_min) or range_min < 0.0:
            range_min = 0.0
        out.range_min = range_min

        range_max = float(msg.range_max)
        if (not math.isfinite(range_max)) or range_max <= range_min:
            range_max = max(self.fallback_range_max, range_min + 1e-3)
            self._warn_invalid_bounds(msg.range_min, msg.range_max, range_max, msg.header.frame_id)
        out.range_max = range_max

        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self.publisher.publish(out)

    def _warn_invalid_bounds(
        self,
        original_range_min: float,
        original_range_max: float,
        sanitized_range_max: float,
        frame_id: str,
    ) -> None:
        now = time.monotonic()
        if now - self.last_warn_time < self.warn_period_sec:
            return
        self.last_warn_time = now
        self.get_logger().warning(
            "Sanitized LaserScan with invalid bounds "
            f"(frame_id={frame_id}, range_min={original_range_min}, "
            f"range_max={original_range_max}, fallback_range_max={sanitized_range_max})"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaserScanSanitizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
