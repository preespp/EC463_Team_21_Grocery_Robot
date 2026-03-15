#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Iterable, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.srv import ResolveSemanticTarget
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _normalize_key(value: str) -> str:
    return str(value or "").strip().lower()


class SemanticMapServer(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_server")

        self.declare_parameter("semantic_map_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_topic", "/semantic_map/markers")
        self.declare_parameter("service_name", "/semantic_map/resolve_target")
        self.declare_parameter("publish_period_sec", 1.0)

        semantic_map_file = str(self.get_parameter("semantic_map_file").value or "").strip()
        self.semantic_map_file = Path(semantic_map_file) if semantic_map_file else None
        self.frame_id = str(self.get_parameter("frame_id").value or "map")
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.service_name = str(self.get_parameter("service_name").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value or 1.0)

        self.map_name = ""
        self.semantic_id = ""
        self.anchors: Dict[str, dict] = {}
        self.racks: Dict[str, dict] = {}
        self.slots: Dict[str, dict] = {}
        self.product_id_index: Dict[str, str] = {}
        self.product_name_index: Dict[str, str] = {}

        self._load_semantic_map()

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.create_service(ResolveSemanticTarget, self.service_name, self._handle_resolve_target)

        if self.publish_markers:
            self.marker_timer = self.create_timer(self.publish_period_sec, self.publish_marker_array)
            self.publish_marker_array()

        self.get_logger().info(
            "Semantic map ready | "
            f"map={self.map_name} semantic_id={self.semantic_id} "
            f"anchors={len(self.anchors)} racks={len(self.racks)} "
            f"slots={len(self.slots)} service={self.service_name}"
        )

    def _load_semantic_map(self) -> None:
        if self.semantic_map_file is None:
            raise RuntimeError("semantic_map_file parameter is required")
        if not self.semantic_map_file.exists():
            raise RuntimeError(f"Semantic map file not found: {self.semantic_map_file}")

        with self.semantic_map_file.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}

        map_section = raw.get("map", {})
        self.map_name = str(map_section.get("name", self.semantic_map_file.stem))
        self.semantic_id = str(map_section.get("semantic_id", f"{self.map_name}_semantic"))
        self.frame_id = str(map_section.get("frame_id", self.frame_id or "map"))

        self.anchors.clear()
        self.racks.clear()
        self.slots.clear()
        self.product_id_index.clear()
        self.product_name_index.clear()

        for anchor in raw.get("anchors", []):
            anchor_id = str(anchor.get("id", "")).strip()
            if not anchor_id:
                continue
            self.anchors[anchor_id] = {
                "id": anchor_id,
                "label": str(anchor.get("label", anchor_id)),
                "type": str(anchor.get("type", "anchor")),
                "x": float(anchor.get("x", 0.0)),
                "y": float(anchor.get("y", 0.0)),
                "yaw": float(anchor.get("yaw", 0.0)),
            }

        for rack in raw.get("racks", []):
            rack_id = str(rack.get("id", "")).strip()
            if not rack_id:
                continue
            self.racks[rack_id] = {
                "id": rack_id,
                "label": str(rack.get("label", rack_id)),
                "anchor_id": str(rack.get("anchor_id", "")),
                "x": float(rack.get("x", 0.0)),
                "y": float(rack.get("y", 0.0)),
                "yaw": float(rack.get("yaw", 0.0)),
                "width": float(rack.get("width", 0.8)),
                "depth": float(rack.get("depth", 0.4)),
                "levels": int(rack.get("levels", 1)),
            }

        for slot in raw.get("slots", []):
            slot_id = str(slot.get("id", "")).strip()
            if not slot_id:
                continue
            slot_data = {
                "id": slot_id,
                "label": str(slot.get("label", slot_id)),
                "rack_id": str(slot.get("rack_id", "")),
                "anchor_id": str(slot.get("anchor_id", "")),
                "rack_level": int(slot.get("rack_level", 0)),
                "nav_pose": dict(slot.get("nav_pose", {})),
                "service_pose": dict(slot.get("service_pose", {})),
                "product_ids": [str(v).strip() for v in slot.get("product_ids", []) if str(v).strip()],
                "product_names": [
                    _normalize_key(str(v)) for v in slot.get("product_names", []) if str(v).strip()
                ],
            }

            if not slot_data["nav_pose"] and slot_data["anchor_id"] in self.anchors:
                anchor = self.anchors[slot_data["anchor_id"]]
                slot_data["nav_pose"] = {
                    "x": anchor["x"],
                    "y": anchor["y"],
                    "yaw": anchor["yaw"],
                }

            self.slots[slot_id] = slot_data

            for product_id in slot_data["product_ids"]:
                self.product_id_index[_normalize_key(product_id)] = slot_id
            for product_name in slot_data["product_names"]:
                self.product_name_index[_normalize_key(product_name)] = slot_id

    def _pose_from_dict(self, pose_data: dict, fallback_yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        x = float(pose_data.get("x", 0.0))
        y = float(pose_data.get("y", 0.0))
        z = float(pose_data.get("z", 0.0))
        yaw = float(pose_data.get("yaw", fallback_yaw))
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _resolve_slot(self, slot_id: str) -> Tuple[str, dict] | Tuple[None, None]:
        slot = self.slots.get(slot_id)
        if slot is None:
            return None, None
        return slot_id, slot

    def _resolve_target(self, request: ResolveSemanticTarget.Request) -> dict:
        slot_id = str(request.slot_id or "").strip()
        anchor_id = str(request.anchor_id or "").strip()
        product_id = _normalize_key(request.product_id)
        product_name = _normalize_key(request.product_name)

        if slot_id:
            normalized_slot, slot = self._resolve_slot(slot_id)
            if slot is None:
                return {"found": False, "message": f"slot_id not found: {slot_id}"}
            return {"found": True, "resolved_by": "slot_id", "slot": slot}

        if anchor_id:
            anchor = self.anchors.get(anchor_id)
            if anchor is None:
                return {"found": False, "message": f"anchor_id not found: {anchor_id}"}
            return {"found": True, "resolved_by": "anchor_id", "anchor": anchor}

        if product_id:
            indexed_slot_id = self.product_id_index.get(product_id)
            if indexed_slot_id:
                return {
                    "found": True,
                    "resolved_by": "product_id",
                    "slot": self.slots[indexed_slot_id],
                }

        if product_name:
            indexed_slot_id = self.product_name_index.get(product_name)
            if indexed_slot_id:
                return {
                    "found": True,
                    "resolved_by": "product_name",
                    "slot": self.slots[indexed_slot_id],
                }

        return {
            "found": False,
            "message": (
                "No semantic target found. Provide slot_id, anchor_id, product_id, or product_name."
            ),
        }

    def _handle_resolve_target(
        self,
        request: ResolveSemanticTarget.Request,
        response: ResolveSemanticTarget.Response,
    ) -> ResolveSemanticTarget.Response:
        resolution = self._resolve_target(request)
        response.map_name = self.map_name
        response.semantic_id = self.semantic_id
        response.found = bool(resolution.get("found", False))
        response.message = str(resolution.get("message", ""))
        response.resolved_by = str(resolution.get("resolved_by", ""))

        if not response.found:
            return response

        if "slot" in resolution:
            slot = resolution["slot"]
            rack = self.racks.get(slot.get("rack_id", ""), {})
            anchor = self.anchors.get(slot.get("anchor_id", ""), {})

            response.target_label = slot.get("label", "")
            response.rack_id = slot.get("rack_id", "")
            response.slot_id = slot.get("id", "")
            response.anchor_id = slot.get("anchor_id", "")
            response.rack_level = int(slot.get("rack_level", 0))
            response.nav_pose = self._pose_from_dict(
                slot.get("nav_pose", {}),
                fallback_yaw=float(anchor.get("yaw", 0.0)),
            )
            response.service_pose = self._pose_from_dict(
                slot.get("service_pose", slot.get("nav_pose", {})),
                fallback_yaw=float(anchor.get("yaw", 0.0)),
            )
            if not response.message:
                response.message = (
                    f"Resolved slot {slot.get('id', '')} on rack {rack.get('id', '') or response.rack_id}"
                )
            return response

        anchor = resolution["anchor"]
        response.target_label = anchor.get("label", "")
        response.anchor_id = anchor.get("id", "")
        response.nav_pose = self._pose_from_dict(anchor, fallback_yaw=float(anchor.get("yaw", 0.0)))
        response.service_pose = self._pose_from_dict(
            {"x": anchor["x"], "y": anchor["y"], "z": 0.0, "yaw": anchor["yaw"]},
            fallback_yaw=float(anchor.get("yaw", 0.0)),
        )
        if not response.message:
            response.message = f"Resolved anchor {anchor.get('id', '')}"
        return response

    def publish_marker_array(self) -> None:
        marker_array = MarkerArray()
        marker_id = 0

        for anchor in self.anchors.values():
            marker_array.markers.extend(
                self._build_anchor_markers(anchor=anchor, start_id=marker_id)
            )
            marker_id += 2

        for rack in self.racks.values():
            marker_array.markers.extend(self._build_rack_markers(rack=rack, start_id=marker_id))
            marker_id += 2

        for slot in self.slots.values():
            marker_array.markers.extend(self._build_slot_markers(slot=slot, start_id=marker_id))
            marker_id += 2

        self.marker_pub.publish(marker_array)

    def _build_anchor_markers(self, anchor: dict, start_id: int) -> Iterable[Marker]:
        pose = self._pose_from_dict(anchor, fallback_yaw=float(anchor.get("yaw", 0.0)))

        arrow = Marker()
        arrow.header = pose.header
        arrow.ns = "semantic_anchor"
        arrow.id = start_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose.pose
        arrow.scale.x = 0.45
        arrow.scale.y = 0.10
        arrow.scale.z = 0.10
        arrow.color = ColorRGBA(r=0.10, g=0.85, b=0.95, a=0.95)

        text = Marker()
        text.header = pose.header
        text.ns = "semantic_anchor_text"
        text.id = start_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose.pose
        text.pose.position.z = 0.35
        text.scale.z = 0.20
        text.color = ColorRGBA(r=0.90, g=0.98, b=1.0, a=1.0)
        text.text = anchor.get("label", anchor.get("id", "anchor"))
        return [arrow, text]

    def _build_rack_markers(self, rack: dict, start_id: int) -> Iterable[Marker]:
        pose = self._pose_from_dict(rack, fallback_yaw=float(rack.get("yaw", 0.0)))

        cube = Marker()
        cube.header = pose.header
        cube.ns = "semantic_rack"
        cube.id = start_id
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = pose.pose
        cube.pose.position.z = 0.10
        cube.scale.x = float(rack.get("width", 0.8))
        cube.scale.y = float(rack.get("depth", 0.4))
        cube.scale.z = 0.20
        cube.color = ColorRGBA(r=0.96, g=0.77, b=0.22, a=0.40)

        text = Marker()
        text.header = pose.header
        text.ns = "semantic_rack_text"
        text.id = start_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = pose.pose
        text.pose.position.z = 0.38
        text.scale.z = 0.18
        text.color = ColorRGBA(r=1.0, g=0.95, b=0.75, a=1.0)
        text.text = rack.get("label", rack.get("id", "rack"))
        return [cube, text]

    def _build_slot_markers(self, slot: dict, start_id: int) -> Iterable[Marker]:
        nav_pose = self._pose_from_dict(slot.get("nav_pose", {}))

        sphere = Marker()
        sphere.header = nav_pose.header
        sphere.ns = "semantic_slot"
        sphere.id = start_id
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = nav_pose.pose
        sphere.pose.position.z = 0.12
        sphere.scale.x = 0.14
        sphere.scale.y = 0.14
        sphere.scale.z = 0.14
        sphere.color = ColorRGBA(r=0.37, g=0.90, b=0.45, a=0.85)

        text = Marker()
        text.header = nav_pose.header
        text.ns = "semantic_slot_text"
        text.id = start_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = nav_pose.pose
        text.pose.position.z = 0.28
        text.scale.z = 0.14
        text.color = ColorRGBA(r=0.85, g=1.0, b=0.88, a=1.0)
        text.text = f"{slot.get('label', slot.get('id', 'slot'))} L{slot.get('rack_level', 0)}"
        return [sphere, text]


def main() -> None:
    rclpy.init()
    node = SemanticMapServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
