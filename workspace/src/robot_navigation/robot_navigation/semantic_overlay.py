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
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
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


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("marker_topic", "/semantic_overlay/markers")
        self.declare_parameter("query_service", "/semantic_overlay/query_shelf_pose")
        self.declare_parameter("align_service", "/semantic_overlay/set_alignment")
        self.declare_parameter("auto_align_service", "/semantic_overlay/auto_align")
        self.declare_parameter("reload_service", "/semantic_overlay/reload")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("auto_align_retry_sec", 2.0)
        self.declare_parameter("marker_scale_xy", 0.28)
        self.declare_parameter("marker_scale_z", 0.12)
        self.declare_parameter("marker_alpha", 0.85)
        self.declare_parameter("align_tx", 0.0)
        self.declare_parameter("align_ty", 0.0)
        self.declare_parameter("align_yaw", 0.0)
        self.declare_parameter("reference_map_yaml", "")
        self.declare_parameter("auto_align_on_start", False)
        self.declare_parameter("auto_align_min_score", 0.45)
        self.declare_parameter("auto_align_yaw_search_deg", 12.0)
        self.declare_parameter("auto_align_yaw_step_deg", 2.0)
        self.declare_parameter("auto_align_translation_search_m", 1.5)
        self.declare_parameter("auto_align_translation_step_m", 0.2)
        self.declare_parameter("auto_align_sample_stride_cells", 4)
        self.declare_parameter("auto_align_max_reference_points", 3000)
        self.declare_parameter("auto_align_occupied_threshold", 60)
        self.declare_parameter("auto_align_global_fallback", True)
        self.declare_parameter("auto_align_global_yaw_step_deg", 15.0)
        self.declare_parameter("auto_align_global_translation_step_m", 1.0)
        self.declare_parameter("auto_align_global_translation_limit_m", 6.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.shelves_file = Path(str(self.get_parameter("shelves_file").value)).expanduser()
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.query_service_name = str(self.get_parameter("query_service").value)
        self.align_service_name = str(self.get_parameter("align_service").value)
        self.auto_align_service_name = str(self.get_parameter("auto_align_service").value)
        self.reload_service_name = str(self.get_parameter("reload_service").value)
        publish_rate_hz = max(1e-3, float(self.get_parameter("publish_rate_hz").value))
        self.publish_period = 1.0 / publish_rate_hz
        self.auto_align_retry_sec = max(
            0.5, float(self.get_parameter("auto_align_retry_sec").value)
        )

        self.marker_scale_xy = max(0.01, float(self.get_parameter("marker_scale_xy").value))
        self.marker_scale_z = max(0.01, float(self.get_parameter("marker_scale_z").value))
        self.marker_alpha = min(1.0, max(0.1, float(self.get_parameter("marker_alpha").value)))

        self.align_tx = float(self.get_parameter("align_tx").value)
        self.align_ty = float(self.get_parameter("align_ty").value)
        self.align_yaw = float(self.get_parameter("align_yaw").value)
        self.reference_map_yaml = Path(
            str(self.get_parameter("reference_map_yaml").value)
        ).expanduser()
        self.auto_align_on_start = bool(self.get_parameter("auto_align_on_start").value)
        self.auto_align_min_score = min(
            1.0, max(0.05, float(self.get_parameter("auto_align_min_score").value))
        )
        self.auto_align_yaw_search_deg = max(
            0.0, float(self.get_parameter("auto_align_yaw_search_deg").value)
        )
        self.auto_align_yaw_step_deg = max(
            0.1, float(self.get_parameter("auto_align_yaw_step_deg").value)
        )
        self.auto_align_translation_search_m = max(
            0.0, float(self.get_parameter("auto_align_translation_search_m").value)
        )
        self.auto_align_translation_step_m = max(
            0.02, float(self.get_parameter("auto_align_translation_step_m").value)
        )
        self.auto_align_sample_stride_cells = max(
            1, int(self.get_parameter("auto_align_sample_stride_cells").value)
        )
        self.auto_align_max_reference_points = max(
            200, int(self.get_parameter("auto_align_max_reference_points").value)
        )
        self.auto_align_occupied_threshold = max(
            1, min(100, int(self.get_parameter("auto_align_occupied_threshold").value))
        )
        self.auto_align_global_fallback = bool(
            self.get_parameter("auto_align_global_fallback").value
        )
        self.auto_align_global_yaw_step_deg = max(
            1.0, float(self.get_parameter("auto_align_global_yaw_step_deg").value)
        )
        self.auto_align_global_translation_step_m = max(
            0.2, float(self.get_parameter("auto_align_global_translation_step_m").value)
        )
        self.auto_align_global_translation_limit_m = max(
            0.0, float(self.get_parameter("auto_align_global_translation_limit_m").value)
        )

        self.shelves: Dict[str, ShelfPose2D] = {}
        self._last_marker_ids: Set[Tuple[str, int]] = set()
        self._force_deleteall = True
        self.reference_occ_points: List[Tuple[float, float]] = []

        self.map_data: List[int] = []
        self.map_w = 0
        self.map_h = 0
        self.map_res = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_origin_cos = 1.0
        self.map_origin_sin = 0.0

        self.auto_align_pending = self.auto_align_on_start
        self.last_auto_align_attempt_at = 0.0
        self.last_auto_align_warn_at = 0.0

        qos = QoSProfile(depth=10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, qos)

        self.query_srv = self.create_service(
            QueryShelfPose, self.query_service_name, self._on_query_shelf_pose
        )
        self.align_srv = self.create_service(
            SetSemanticAlignment, self.align_service_name, self._on_set_alignment
        )
        self.auto_align_srv = self.create_service(
            Trigger, self.auto_align_service_name, self._on_auto_align
        )
        self.reload_srv = self.create_service(
            Trigger, self.reload_service_name, self._on_reload_shelves
        )

        self.marker_timer = self.create_timer(self.publish_period, self._publish_markers)
        self.auto_align_timer = self.create_timer(self.auto_align_retry_sec, self._auto_align_tick)

        try:
            self._load_shelves_file()
        except Exception as exc:  # pragma: no cover
            self.shelves = {}
            self.get_logger().error(f"Failed to load shelves_file '{self.shelves_file}': {exc}")
        try:
            self._load_reference_map()
        except Exception as exc:  # pragma: no cover
            self.reference_occ_points = []
            self.get_logger().error(
                f"Failed to load reference_map_yaml '{self.reference_map_yaml}': {exc}"
            )
        self.get_logger().info(
            "semantic_overlay ready "
            f"(frame={self.frame_id}, shelves={len(self.shelves)}, file='{self.shelves_file}', "
            f"reference_points={len(self.reference_occ_points)}, auto_align_on_start={self.auto_align_on_start})"
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

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_w = int(msg.info.width)
        self.map_h = int(msg.info.height)
        self.map_res = float(msg.info.resolution)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)
        q = msg.info.origin.orientation
        self.map_origin_yaw = _quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        self.map_origin_cos = math.cos(self.map_origin_yaw)
        self.map_origin_sin = math.sin(self.map_origin_yaw)
        self.map_data = list(msg.data)

    def _on_auto_align(self, _request, response: Trigger.Response) -> Trigger.Response:
        success, message = self._run_auto_alignment_once()
        response.success = success
        response.message = message
        if success:
            self.auto_align_pending = False
            self._publish_markers()
        return response

    def _auto_align_tick(self) -> None:
        if not self.auto_align_pending:
            return
        now = time.monotonic()
        if now - self.last_auto_align_attempt_at < self.auto_align_retry_sec:
            return
        self.last_auto_align_attempt_at = now

        success, message = self._run_auto_alignment_once()
        if success:
            self.auto_align_pending = False
            self.get_logger().info(message)
            self._publish_markers()
            return

        if now - self.last_auto_align_warn_at > 5.0:
            self.get_logger().warning(message)
            self.last_auto_align_warn_at = now

    def _run_auto_alignment_once(self) -> Tuple[bool, str]:
        if not self.reference_occ_points:
            return False, "auto_align skipped: reference occupied pattern is empty"
        if self.map_w <= 0 or self.map_h <= 0 or not self.map_data:
            return False, "auto_align skipped: current /map is not available yet"

        yaw_offsets = self._make_search_offsets(
            self.auto_align_yaw_search_deg, self.auto_align_yaw_step_deg
        )
        trans_offsets = self._make_search_offsets(
            self.auto_align_translation_search_m, self.auto_align_translation_step_m
        )
        if not yaw_offsets or not trans_offsets:
            return False, "auto_align skipped: invalid search-space parameters"

        best_score, best_in_bounds, best_tx, best_ty, best_yaw = self._search_best_alignment(
            yaw_offsets=yaw_offsets,
            tx_offsets=trans_offsets,
            ty_offsets=trans_offsets,
            yaw_center=self.align_yaw,
            tx_center=self.align_tx,
            ty_center=self.align_ty,
        )

        # If local-window search fails, run a coarse global fallback then refine once.
        if best_score < self.auto_align_min_score and self.auto_align_global_fallback:
            global_yaw_offsets = self._make_search_offsets(
                180.0, self.auto_align_global_yaw_step_deg
            )
            global_trans_offsets = self._make_search_offsets(
                self.auto_align_global_translation_limit_m,
                self.auto_align_global_translation_step_m,
            )
            (
                coarse_score,
                coarse_in_bounds,
                coarse_tx,
                coarse_ty,
                coarse_yaw,
            ) = self._search_best_alignment(
                yaw_offsets=global_yaw_offsets,
                tx_offsets=global_trans_offsets,
                ty_offsets=global_trans_offsets,
                yaw_center=0.0,
                tx_center=0.0,
                ty_center=0.0,
            )
            if coarse_score > best_score:
                best_score = coarse_score
                best_in_bounds = coarse_in_bounds
                best_tx = coarse_tx
                best_ty = coarse_ty
                best_yaw = coarse_yaw

            if coarse_score > 0.0:
                (
                    refined_score,
                    refined_in_bounds,
                    refined_tx,
                    refined_ty,
                    refined_yaw,
                ) = self._search_best_alignment(
                    yaw_offsets=yaw_offsets,
                    tx_offsets=trans_offsets,
                    ty_offsets=trans_offsets,
                    yaw_center=coarse_yaw,
                    tx_center=coarse_tx,
                    ty_center=coarse_ty,
                )
                if refined_score > best_score:
                    best_score = refined_score
                    best_in_bounds = refined_in_bounds
                    best_tx = refined_tx
                    best_ty = refined_ty
                    best_yaw = refined_yaw

        if best_score < self.auto_align_min_score:
            return (
                False,
                f"auto_align failed: best_score={best_score:.3f} "
                f"< min_score={self.auto_align_min_score:.3f}",
            )

        self.align_tx = best_tx
        self.align_ty = best_ty
        self.align_yaw = best_yaw
        return (
            True,
            f"auto_align success: score={best_score:.3f}, in_bounds={best_in_bounds}, "
            f"tx={self.align_tx:.3f}, ty={self.align_ty:.3f}, yaw={self.align_yaw:.3f}",
        )

    def _search_best_alignment(
        self,
        yaw_offsets: Sequence[float],
        tx_offsets: Sequence[float],
        ty_offsets: Sequence[float],
        yaw_center: float,
        tx_center: float,
        ty_center: float,
    ) -> Tuple[float, int, float, float, float]:
        best_score = -1.0
        best_in_bounds = -1
        best_tx = tx_center
        best_ty = ty_center
        best_yaw = yaw_center
        for yaw_offset in yaw_offsets:
            yaw = _norm_angle(yaw_center + math.radians(float(yaw_offset)))
            for tx_offset in tx_offsets:
                tx = tx_center + float(tx_offset)
                for ty_offset in ty_offsets:
                    ty = ty_center + float(ty_offset)
                    score, in_bounds = self._score_alignment(tx, ty, yaw, self.reference_occ_points)
                    if score > best_score or (
                        math.isclose(score, best_score) and in_bounds > best_in_bounds
                    ):
                        best_score = score
                        best_in_bounds = in_bounds
                        best_tx = tx
                        best_ty = ty
                        best_yaw = yaw
        return (best_score, best_in_bounds, best_tx, best_ty, best_yaw)

    def _score_alignment(
        self, tx: float, ty: float, yaw: float, points: Sequence[Tuple[float, float]]
    ) -> Tuple[float, int]:
        if not points:
            return (0.0, 0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        matched = 0
        in_bounds = 0
        for ref_x, ref_y in points:
            wx = cos_yaw * ref_x - sin_yaw * ref_y + tx
            wy = sin_yaw * ref_x + cos_yaw * ref_y + ty
            map_idx = self._current_world_to_map_idx(wx, wy)
            if map_idx is None:
                continue
            in_bounds += 1
            if self.map_data[map_idx] >= self.auto_align_occupied_threshold:
                matched += 1
        if in_bounds <= 0:
            return (0.0, 0)
        return (float(matched) / float(in_bounds), in_bounds)

    def _current_world_to_map_idx(self, wx: float, wy: float) -> int | None:
        if self.map_w <= 0 or self.map_h <= 0 or self.map_res <= 0.0:
            return None
        dx = wx - self.map_origin_x
        dy = wy - self.map_origin_y
        local_x = self.map_origin_cos * dx + self.map_origin_sin * dy
        local_y = -self.map_origin_sin * dx + self.map_origin_cos * dy
        mx = int(math.floor(local_x / self.map_res))
        my = int(math.floor(local_y / self.map_res))
        if mx < 0 or my < 0 or mx >= self.map_w or my >= self.map_h:
            return None
        return my * self.map_w + mx

    def _make_search_offsets(self, search_range: float, step: float) -> List[float]:
        if search_range <= 0.0:
            return [0.0]
        step = max(1e-6, step)
        result: List[float] = []
        current = -search_range
        while current <= search_range + 1e-9:
            result.append(current)
            current += step
        if not result:
            return [0.0]
        return result

    def _load_reference_map(self) -> None:
        self.reference_occ_points = []
        if not self.reference_map_yaml or str(self.reference_map_yaml) in ("", "."):
            return
        if not self.reference_map_yaml.exists():
            self.get_logger().warning(
                f"reference_map_yaml not found: {self.reference_map_yaml}"
            )
            return
        if yaml is None:
            raise RuntimeError(
                "PyYAML not available. Install python3-yaml to load reference_map_yaml."
            )

        config = yaml.safe_load(self.reference_map_yaml.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise RuntimeError("reference_map_yaml must contain a yaml mapping")
        image_rel = str(config.get("image", "")).strip()
        if not image_rel:
            raise RuntimeError("reference_map_yaml missing 'image' field")
        image_path = Path(image_rel)
        if not image_path.is_absolute():
            image_path = (self.reference_map_yaml.parent / image_path).resolve()

        width, height, pixels = self._load_pgm(image_path)
        resolution = float(config.get("resolution", 0.05))
        if resolution <= 0.0:
            raise RuntimeError("reference map resolution must be > 0")

        origin = config.get("origin", [0.0, 0.0, 0.0])
        if not isinstance(origin, list) or len(origin) < 3:
            raise RuntimeError("reference map origin must be [x, y, yaw]")
        origin_x = float(origin[0])
        origin_y = float(origin[1])
        origin_yaw = float(origin[2])
        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)
        occupied_threshold = float(config.get("occupied_thresh", 0.65))
        negate = int(config.get("negate", 0))

        points: List[Tuple[float, float]] = []
        stride = self.auto_align_sample_stride_cells
        for py in range(0, height, stride):
            for px in range(0, width, stride):
                pixel = pixels[py * width + px]
                occupancy = float(pixel) / 255.0 if negate else float(255 - pixel) / 255.0
                if occupancy < occupied_threshold:
                    continue
                map_x = float(px) + 0.5
                map_y = float(height - 1 - py) + 0.5
                local_x = map_x * resolution
                local_y = map_y * resolution
                wx = origin_x + origin_cos * local_x - origin_sin * local_y
                wy = origin_y + origin_sin * local_x + origin_cos * local_y
                points.append((wx, wy))

        if not points:
            self.get_logger().warning(
                f"reference_map_yaml '{self.reference_map_yaml}' has no occupied points"
            )
            self.reference_occ_points = []
            return

        if len(points) > self.auto_align_max_reference_points:
            step = max(1, len(points) // self.auto_align_max_reference_points)
            points = points[::step][: self.auto_align_max_reference_points]
        self.reference_occ_points = points
        self.get_logger().info(
            f"Loaded reference map pattern: {len(self.reference_occ_points)} occupied samples"
        )

    def _load_pgm(self, path: Path) -> Tuple[int, int, List[int]]:
        if not path.exists():
            raise RuntimeError(f"reference map image not found: {path}")
        with path.open("rb") as f:
            magic = self._read_pgm_token(f)
            if magic not in ("P5", "P2"):
                raise RuntimeError(f"unsupported map image format '{magic}' (need PGM P5/P2)")
            width = int(self._read_pgm_token(f))
            height = int(self._read_pgm_token(f))
            maxval = int(self._read_pgm_token(f))
            if maxval <= 0 or maxval > 255:
                raise RuntimeError(f"unsupported PGM max value: {maxval}")
            total = width * height
            if magic == "P5":
                data = f.read(total)
                if len(data) < total:
                    raise RuntimeError("invalid PGM: not enough pixel bytes")
                return (width, height, list(data[:total]))

            pixels: List[int] = []
            for _ in range(total):
                token = self._read_pgm_token(f)
                if token is None:
                    raise RuntimeError("invalid PGM: missing ASCII pixels")
                pixels.append(int(token))
            return (width, height, pixels)

    def _read_pgm_token(self, stream) -> str | None:
        token = bytearray()
        while True:
            ch = stream.read(1)
            if not ch:
                if token:
                    return token.decode("ascii")
                return None
            if ch == b"#":
                stream.readline()
                if token:
                    return token.decode("ascii")
                continue
            if ch in b" \t\r\n":
                if token:
                    return token.decode("ascii")
                continue
            token.extend(ch)

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
        self.auto_align_pending = False
        self._publish_markers()
        return response

    def _on_reload_shelves(self, _request, response: Trigger.Response) -> Trigger.Response:
        try:
            self._load_shelves_file()
            self._load_reference_map()
        except Exception as exc:  # pragma: no cover
            response.success = False
            response.message = f"reload failed: {exc}"
            self.get_logger().error(response.message)
            return response
        self._force_deleteall = True
        if self.auto_align_on_start:
            self.auto_align_pending = True
        self._publish_markers()
        response.success = True
        response.message = (
            f"loaded {len(self.shelves)} shelves, "
            f"reference_points={len(self.reference_occ_points)}"
        )
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
