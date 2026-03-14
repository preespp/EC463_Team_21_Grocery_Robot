#!/usr/bin/env python3
"""
Crop PointCloud2 returns that fall inside a box defined in a target frame.

This is useful for removing self-hits from the robot chassis before Cartographer
consumes the cloud.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


class BaseLinkCropFilter(Node):
    def __init__(self) -> None:
        super().__init__("base_link_crop_filter")

        self.declare_parameter("enabled", True)
        self.declare_parameter("input_topic", "/cloud_all_fields_fullframe")
        self.declare_parameter("output_topic", "/cloud_all_fields_fullframe_filtered")
        self.declare_parameter("box_frame", "base_link")
        self.declare_parameter("min_x", -0.2540)
        self.declare_parameter("max_x", 0.1397)
        self.declare_parameter("min_y", -0.2794)
        self.declare_parameter("max_y", 0.2794)
        self.declare_parameter("min_z", -1.0)
        self.declare_parameter("max_z", 1.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.box_frame = str(self.get_parameter("box_frame").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.min_y = float(self.get_parameter("min_y").value)
        self.max_y = float(self.get_parameter("max_y").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, 10
        )

        self.last_warn_time = 0.0
        self.last_stats_time = 0.0

        self.get_logger().info(
            f"Base-link crop filter ready: enabled={self.enabled}, "
            f"{self.input_topic} -> {self.output_topic}, "
            f"frame={self.box_frame}, "
            f"box=({self.min_x:.3f},{self.max_x:.3f}) x "
            f"({self.min_y:.3f},{self.max_y:.3f}) x "
            f"({self.min_z:.3f},{self.max_z:.3f})"
        )

    def _warn_throttled(self, message: str, period_sec: float = 2.0) -> None:
        now = time.monotonic()
        if (now - self.last_warn_time) >= period_sec:
            self.get_logger().warning(message)
            self.last_warn_time = now

    def _log_stats_throttled(
        self, total_points: int, removed_points: int, period_sec: float = 2.0
    ) -> None:
        now = time.monotonic()
        if (now - self.last_stats_time) >= period_sec:
            kept_points = total_points - removed_points
            self.get_logger().info(
                f"Crop stats: removed {removed_points}/{total_points} points, "
                f"kept {kept_points}, frame={self.box_frame}"
            )
            self.last_stats_time = now

    def _lookup_transform(self, msg: PointCloud2):
        stamp = Time.from_msg(msg.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(self.box_frame, msg.header.frame_id, stamp)
        except TransformException:
            try:
                return self.tf_buffer.lookup_transform(self.box_frame, msg.header.frame_id, Time())
            except TransformException as exc:
                self._warn_throttled(
                    f"Cannot transform {msg.header.frame_id} -> {self.box_frame}: {exc}"
                )
                return None

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if not self.enabled:
            self.publisher.publish(msg)
            return

        transform = self._lookup_transform(msg)
        if transform is None:
            return

        points = point_cloud2.read_points(msg, skip_nans=False)
        if points.size == 0:
            self.publisher.publish(msg)
            return

        names = points.dtype.names or ()
        if not {"x", "y", "z"}.issubset(names):
            self._warn_throttled("Incoming cloud is missing x/y/z fields; passing through.")
            self.publisher.publish(msg)
            return

        valid_xyz = (
            np.isfinite(points["x"]) & np.isfinite(points["y"]) & np.isfinite(points["z"])
        )
        if not np.any(valid_xyz):
            self.publisher.publish(msg)
            return

        xyz = np.stack(
            (points["x"][valid_xyz], points["y"][valid_xyz], points["z"][valid_xyz]), axis=1
        ).astype(np.float32, copy=False)

        q = transform.transform.rotation
        t = transform.transform.translation
        rotation = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        translation = np.array([t.x, t.y, t.z], dtype=np.float32)

        # Evaluate each input point in box_frame, but keep the cloud published in
        # its original frame so downstream consumers do not need any remapping.
        xyz_in_box_frame = xyz @ rotation.T + translation

        inside_valid = (
            (xyz_in_box_frame[:, 0] >= self.min_x)
            & (xyz_in_box_frame[:, 0] <= self.max_x)
            & (xyz_in_box_frame[:, 1] >= self.min_y)
            & (xyz_in_box_frame[:, 1] <= self.max_y)
            & (xyz_in_box_frame[:, 2] >= self.min_z)
            & (xyz_in_box_frame[:, 2] <= self.max_z)
        )

        remove_mask = np.zeros(points.shape[0], dtype=bool)
        remove_mask[np.nonzero(valid_xyz)[0]] = inside_valid
        filtered_points = points[~remove_mask]
        self._log_stats_throttled(
            total_points=int(points.shape[0]),
            removed_points=int(np.count_nonzero(remove_mask)),
        )

        filtered_cloud = point_cloud2.create_cloud(msg.header, msg.fields, filtered_points)
        filtered_cloud.is_bigendian = msg.is_bigendian
        filtered_cloud.is_dense = bool(msg.is_dense and not np.any(remove_mask))
        self.publisher.publish(filtered_cloud)


def main() -> int:
    rclpy.init()
    node = BaseLinkCropFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
