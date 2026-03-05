#!/usr/bin/env python3
"""
Semantic shelf overlay for mapped environments.

Features:
- Load shelves from YAML/JSON (`shelf_id -> pose` in reference map frame).
- Apply rigid transform from reference-map to current-map.
- Publish transformed shelf markers.
- Provide shelf pose query service.
- Provide two-point manual alignment service.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.utilities import ok as rclpy_ok
from robot_interfaces.srv import QueryShelfPose, SetSemanticAlignment
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def _norm_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass
class ShelfPose2D:
    shelf_id: str
    x: float
    y: float
    yaw: float


class SemanticOverlay(Node):
    """Publish/query semantic shelves aligned to current map frame."""

    def __init__(self) -> None:
        super().__init__("semantic_overlay")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("shelves_file", "")
        self.declare_parameter("marker_topic", "/semantic_overlay/markers")
        self.declare_parameter("query_service", "/semantic_overlay/query_shelf_pose")
        self.declare_parameter("align_service", "/semantic_overlay/set_alignment")
        self.declare_parameter("reload_service", "/semantic_overlay/reload")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("marker_scale_xy", 0.28)
        self.declare_parameter("marker_scale_z", 0.12)
        self.declare_parameter("marker_alpha", 0.85)
        self.declare_parameter("align_tx", 0.0)
        self.declare_parameter("align_ty", 0.0)
        self.declare_parameter("align_yaw", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.shelves_file = Path(str(self.get_parameter("shelves_file").value)).expanduser()
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.query_service_name = str(self.get_parameter("query_service").value)
        self.align_service_name = str(self.get_parameter("align_service").value)
        self.reload_service_name = str(self.get_parameter("reload_service").value)
        publish_rate_hz = max(1e-3, float(self.get_parameter("publish_rate_hz").value))
        self.publish_period = 1.0 / publish_rate_hz

        self.marker_scale_xy = max(0.01, float(self.get_parameter("marker_scale_xy").value))
        self.marker_scale_z = max(0.01, float(self.get_parameter("marker_scale_z").value))
        self.marker_alpha = min(1.0, max(0.1, float(self.get_parameter("marker_alpha").value)))

        self.align_tx = float(self.get_parameter("align_tx").value)
        self.align_ty = float(self.get_parameter("align_ty").value)
        self.align_yaw = float(self.get_parameter("align_yaw").value)

        self.shelves: Dict[str, ShelfPose2D] = {}
        self._last_marker_ids: Set[Tuple[str, int]] = set()
        self._force_deleteall = True

        qos = QoSProfile(depth=10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)

        self.query_srv = self.create_service(
            QueryShelfPose, self.query_service_name, self._on_query_shelf_pose
        )
        self.align_srv = self.create_service(
            SetSemanticAlignment, self.align_service_name, self._on_set_alignment
        )
        self.reload_srv = self.create_service(
            Trigger, self.reload_service_name, self._on_reload_shelves
        )

        self.marker_timer = self.create_timer(self.publish_period, self._publish_markers)

        try:
            self._load_shelves_file()
        except Exception as exc:  # pragma: no cover
            self.shelves = {}
            self.get_logger().error(f"Failed to load shelves_file '{self.shelves_file}': {exc}")
        self.get_logger().info(
            "semantic_overlay ready "
            f"(frame={self.frame_id}, shelves={len(self.shelves)}, file='{self.shelves_file}')"
        )

    def _load_shelves_file(self) -> None:
        if not self.shelves_file or str(self.shelves_file) in ("", "."):
            self.get_logger().warning("No shelves_file configured; semantic layer is empty.")
            self.shelves = {}
            return
        if not self.shelves_file.exists():
            self.get_logger().warning(f"shelves_file not found: {self.shelves_file}")
            self.shelves = {}
            return

        parsed = self._parse_file(self.shelves_file)
        shelves: Dict[str, ShelfPose2D] = {}
        for entry in parsed:
            shelf_id = str(entry["shelf_id"]).strip()
            if not shelf_id:
                continue
            shelves[shelf_id] = ShelfPose2D(
                shelf_id=shelf_id,
                x=float(entry["x"]),
                y=float(entry["y"]),
                yaw=float(entry.get("yaw", 0.0)),
            )

        self.shelves = shelves
        self.get_logger().info(f"Loaded {len(self.shelves)} shelves from {self.shelves_file}")

    def _parse_file(self, path: Path) -> List[dict]:
        suffix = path.suffix.lower()
        raw: object
        if suffix == ".json":
            raw = json.loads(path.read_text(encoding="utf-8"))
        else:
            if yaml is None:
                raise RuntimeError(
                    "PyYAML not available. Install python3-yaml or use JSON shelves_file."
                )
            raw = yaml.safe_load(path.read_text(encoding="utf-8"))

        if raw is None:
            return []

        # Supported formats:
        # 1) shelves: [ {shelf_id, x, y, yaw}, ... ]
        # 2) [ {shelf_id, x, y, yaw}, ... ]
        # 3) { shelf_a: {x,y,yaw}, shelf_b: ... }
        if isinstance(raw, dict) and "shelves" in raw:
            raw = raw["shelves"]

        if isinstance(raw, list):
            result: List[dict] = []
            for item in raw:
                if not isinstance(item, dict):
                    continue
                if "pose" in item and isinstance(item["pose"], dict):
                    pose = item["pose"]
                    result.append(
                        {
                            "shelf_id": item.get("shelf_id", item.get("id", "")),
                            "x": pose.get("x", 0.0),
                            "y": pose.get("y", 0.0),
                            "yaw": pose.get("yaw", 0.0),
                        }
                    )
                    continue
                result.append(
                    {
                        "shelf_id": item.get("shelf_id", item.get("id", "")),
                        "x": item.get("x", 0.0),
                        "y": item.get("y", 0.0),
                        "yaw": item.get("yaw", 0.0),
                    }
                )
            return result

        if isinstance(raw, dict):
            result = []
            for shelf_id, pose in raw.items():
                if not isinstance(pose, dict):
                    continue
                result.append(
                    {
                        "shelf_id": shelf_id,
                        "x": pose.get("x", 0.0),
                        "y": pose.get("y", 0.0),
                        "yaw": pose.get("yaw", 0.0),
                    }
                )
            return result

        return []

    def _transform_pose(self, ref_pose: ShelfPose2D) -> ShelfPose2D:
        cos_yaw = math.cos(self.align_yaw)
        sin_yaw = math.sin(self.align_yaw)
        x = cos_yaw * ref_pose.x - sin_yaw * ref_pose.y + self.align_tx
        y = sin_yaw * ref_pose.x + cos_yaw * ref_pose.y + self.align_ty
        yaw = _norm_angle(ref_pose.yaw + self.align_yaw)
        return ShelfPose2D(ref_pose.shelf_id, x, y, yaw)

    def _pose_to_stamped(self, pose: ShelfPose2D) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        qx, qy, qz, qw = _yaw_to_quaternion(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _on_query_shelf_pose(
        self, request: QueryShelfPose.Request, response: QueryShelfPose.Response
    ) -> QueryShelfPose.Response:
        shelf_id = str(request.shelf_id).strip()
        if shelf_id not in self.shelves:
            response.found = False
            response.pose = PoseStamped()
            response.message = f"shelf_id '{shelf_id}' not found"
            return response

        transformed = self._transform_pose(self.shelves[shelf_id])
        response.found = True
        response.pose = self._pose_to_stamped(transformed)
        response.message = "ok"
        return response

    def _on_set_alignment(
        self, request: SetSemanticAlignment.Request, response: SetSemanticAlignment.Response
    ) -> SetSemanticAlignment.Response:
        ref_dx = float(request.ref_x2) - float(request.ref_x1)
        ref_dy = float(request.ref_y2) - float(request.ref_y1)
        new_dx = float(request.new_x2) - float(request.new_x1)
        new_dy = float(request.new_y2) - float(request.new_y1)

        ref_norm = math.hypot(ref_dx, ref_dy)
        new_norm = math.hypot(new_dx, new_dy)
        if ref_norm < 1e-6 or new_norm < 1e-6:
            response.success = False
            response.message = "invalid points: distance between point pairs is too small"
            return response

        yaw = math.atan2(new_dy, new_dx) - math.atan2(ref_dy, ref_dx)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        ref_x1 = float(request.ref_x1)
        ref_y1 = float(request.ref_y1)
        new_x1 = float(request.new_x1)
        new_y1 = float(request.new_y1)

        tx = new_x1 - (cos_yaw * ref_x1 - sin_yaw * ref_y1)
        ty = new_y1 - (sin_yaw * ref_x1 + cos_yaw * ref_y1)

        self.align_tx = tx
        self.align_ty = ty
        self.align_yaw = _norm_angle(yaw)

        response.success = True
        response.message = (
            f"alignment set: tx={self.align_tx:.3f}, ty={self.align_ty:.3f}, "
            f"yaw={self.align_yaw:.3f} rad"
        )
        self.get_logger().info(response.message)
        self._publish_markers()
        return response

    def _on_reload_shelves(self, _request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._load_shelves_file()
        except Exception as exc:  # pragma: no cover
            response.success = False
            response.message = f"reload failed: {exc}"
            self.get_logger().error(response.message)
            return response
        self._force_deleteall = True
        self._publish_markers()
        response.success = True
        response.message = f"loaded {len(self.shelves)} shelves"
        return response

    def _make_delete_all_marker(self, stamp) -> Marker:
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        return clear

    def _publish_markers(self) -> None:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if self._force_deleteall:
            marker_array.markers.append(self._make_delete_all_marker(stamp))

        marker_id = 0
        current_marker_ids: Set[Tuple[str, int]] = set()
        for shelf in self.shelves.values():
            transformed = self._transform_pose(shelf)
            qx, qy, qz, qw = _yaw_to_quaternion(transformed.yaw)

            shape = Marker()
            shape.header.frame_id = self.frame_id
            shape.header.stamp = stamp
            shape.ns = "semantic_shelves"
            shape.id = marker_id
            marker_id += 1
            shape.type = Marker.CUBE
            shape.action = Marker.ADD
            shape.pose.position.x = transformed.x
            shape.pose.position.y = transformed.y
            shape.pose.position.z = self.marker_scale_z * 0.5
            shape.pose.orientation.x = qx
            shape.pose.orientation.y = qy
            shape.pose.orientation.z = qz
            shape.pose.orientation.w = qw
            shape.scale.x = self.marker_scale_xy
            shape.scale.y = self.marker_scale_xy
            shape.scale.z = self.marker_scale_z
            shape.color.r = 0.12
            shape.color.g = 0.62
            shape.color.b = 0.95
            shape.color.a = self.marker_alpha
            marker_array.markers.append(shape)
            current_marker_ids.add((shape.ns, shape.id))

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "semantic_shelf_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = transformed.x
            label.pose.position.y = transformed.y
            label.pose.position.z = self.marker_scale_z + 0.15
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = shelf.shelf_id
            marker_array.markers.append(label)
            current_marker_ids.add((label.ns, label.id))

        stale_markers = self._last_marker_ids - current_marker_ids
        for ns, marker_id in sorted(stale_markers):
            stale = Marker()
            stale.header.frame_id = self.frame_id
            stale.header.stamp = stamp
            stale.ns = ns
            stale.id = marker_id
            stale.action = Marker.DELETE
            marker_array.markers.append(stale)

        self.marker_pub.publish(marker_array)
        self._last_marker_ids = current_marker_ids
        self._force_deleteall = False

    def destroy_node(self) -> bool:
        marker_array = MarkerArray()
        marker_array.markers.append(self._make_delete_all_marker(self.get_clock().now().to_msg()))
        self.marker_pub.publish(marker_array)
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticOverlay()
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
