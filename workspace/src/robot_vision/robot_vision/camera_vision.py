import json
import math
import os
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from ultralytics import YOLO, YOLOE
from ultralytics.utils import SETTINGS
from ultralytics.utils.downloads import attempt_download_asset


def median_depth(depth_frame, x, y, k=5):
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k // 2), max(0, y - k // 2)
    x1, y1 = min(w - 1, x + k // 2), min(h - 1, y + k // 2)
    vals = []
    for yy in range(y0, y1 + 1):
        for xx in range(x0, x1 + 1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0


def nearest_valid_pixel(
    depth_frame,
    x_start,
    y_start,
    x_end,
    y_end,
    target_x,
    target_y,
    *,
    step=2,
    min_depth_m=0.08,
    max_depth_m=3.0,
    reference_depth_m=None,
    reference_tolerance_m=0.10,
):
    best_pixel = None
    best_score = None
    for yy in range(y_start, y_end + 1, step):
        for xx in range(x_start, x_end + 1, step):
            d = depth_frame.get_distance(xx, yy)
            if not (min_depth_m < d < max_depth_m):
                continue
            if (
                reference_depth_m is not None
                and abs(float(d) - float(reference_depth_m)) > float(reference_tolerance_m)
            ):
                continue
            depth_local = median_depth(depth_frame, xx, yy, k=5)
            if depth_local <= 0.0:
                continue
            score = (xx - target_x) ** 2 + (yy - target_y) ** 2
            if best_score is None or score < best_score:
                best_score = score
                best_pixel = (xx, yy, depth_local)
    return best_pixel


def robust_depth_near_pixel(
    depth_frame,
    target_x,
    target_y,
    *,
    bbox=None,
    local_half_window_px=12,
    reference_depth_m=None,
    reference_tolerance_m=0.10,
):
    h, w = depth_frame.get_height(), depth_frame.get_width()
    tx = int(max(0, min(w - 1, target_x)))
    ty = int(max(0, min(h - 1, target_y)))

    center_depth = median_depth(depth_frame, tx, ty, k=7)
    if center_depth > 0.0:
        return center_depth, tx, ty

    wx1 = max(0, tx - int(local_half_window_px))
    wy1 = max(0, ty - int(local_half_window_px))
    wx2 = min(w - 1, tx + int(local_half_window_px))
    wy2 = min(h - 1, ty + int(local_half_window_px))
    local_pick = nearest_valid_pixel(
        depth_frame,
        wx1,
        wy1,
        wx2,
        wy2,
        tx,
        ty,
        step=1,
        reference_depth_m=reference_depth_m,
        reference_tolerance_m=reference_tolerance_m,
    )
    if local_pick is not None:
        px, py, depth_m = local_pick
        return depth_m, px, py

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        bx1 = max(0, min(w - 1, int(x1)))
        by1 = max(0, min(h - 1, int(y1)))
        bx2 = max(0, min(w - 1, int(x2)))
        by2 = max(0, min(h - 1, int(y2)))
        bbox_pick = nearest_valid_pixel(
            depth_frame,
            bx1,
            by1,
            bx2,
            by2,
            tx,
            ty,
            step=2,
            reference_depth_m=reference_depth_m,
            reference_tolerance_m=reference_tolerance_m,
        )
        if bbox_pick is not None:
            px, py, depth_m = bbox_pick
            return depth_m, px, py

    return 0.0, tx, ty


def robust_bbox_depth(depth_frame, x1, y1, x2, y2):
    """
    Get a robust depth estimate near the center of a bbox.
    1) Try median depth at bbox center.
    2) If invalid, find the nearest valid depth pixel in a centered inner window.
    3) If still invalid, find the nearest valid depth pixel in the whole bbox.

    This keeps the grasp target centered when possible, but avoids the physically
    inconsistent case of using the center pixel with depth borrowed from somewhere
    else in the box.
    """
    cx = int((x1 + x2) * 0.5)
    cy = int((y1 + y2) * 0.5)
    center_depth = median_depth(depth_frame, cx, cy, k=7)
    if center_depth > 0.0:
        return center_depth, cx, cy

    h, w = depth_frame.get_height(), depth_frame.get_width()
    bx1 = max(0, min(w - 1, x1))
    by1 = max(0, min(h - 1, y1))
    bx2 = max(0, min(w - 1, x2))
    by2 = max(0, min(h - 1, y2))
    if bx2 <= bx1 or by2 <= by1:
        return 0.0, cx, cy

    width = bx2 - bx1 + 1
    height = by2 - by1 + 1

    inner_half_w = max(2, width // 4)
    inner_half_h = max(2, height // 4)
    ix1 = max(bx1, cx - inner_half_w)
    iy1 = max(by1, cy - inner_half_h)
    ix2 = min(bx2, cx + inner_half_w)
    iy2 = min(by2, cy + inner_half_h)

    inner_pick = nearest_valid_pixel(depth_frame, ix1, iy1, ix2, iy2, cx, cy)
    if inner_pick is not None:
        px, py, depth_m = inner_pick
        return depth_m, px, py

    bbox_pick = nearest_valid_pixel(depth_frame, bx1, by1, bx2, by2, cx, cy)
    if bbox_pick is not None:
        px, py, depth_m = bbox_pick
        return depth_m, px, py

    return 0.0, cx, cy


def band_horizontal_span(
    depth_frame,
    x1,
    y1,
    x2,
    y2,
    *,
    reference_depth_m,
    reference_tolerance_m=0.08,
):
    best_span = 0
    for yy in range(int(y1), int(y2) + 1, 2):
        xs = []
        for xx in range(int(x1), int(x2) + 1, 2):
            d = depth_frame.get_distance(xx, yy)
            if not (0.08 < d < 3.0):
                continue
            if abs(float(d) - float(reference_depth_m)) > float(reference_tolerance_m):
                continue
            xs.append(xx)
        if len(xs) >= 2:
            best_span = max(best_span, int(max(xs) - min(xs)))
    return best_span


def infer_partial_bottle_visibility(
    depth_frame,
    x1,
    y1,
    x2,
    y2,
    *,
    surface_depth_m,
    fy,
    image_height,
    nominal_height_m,
    partial_ratio_threshold,
    edge_contact_px,
):
    bbox_height_px = max(1, int(y2) - int(y1) + 1)
    top_edge_contact = int(y1) <= int(edge_contact_px)
    bottom_edge_contact = int(y2) >= int(image_height) - 1 - int(edge_contact_px)

    if top_edge_contact and not bottom_edge_contact:
        return {
            "partial": True,
            "case": "lower_half",
            "visible_ratio_estimate": 0.0,
            "expected_height_px": 0.0,
            "top_span_px": 0,
            "bottom_span_px": 0,
        }

    if bottom_edge_contact and not top_edge_contact:
        return {
            "partial": True,
            "case": "upper_half",
            "visible_ratio_estimate": 0.0,
            "expected_height_px": 0.0,
            "top_span_px": 0,
            "bottom_span_px": 0,
        }

    if surface_depth_m <= 0.0 or fy <= 0.0 or nominal_height_m <= 0.0:
        return {
            "partial": False,
            "case": "full_or_unknown",
            "visible_ratio_estimate": 1.0,
            "expected_height_px": 0.0,
            "top_span_px": 0,
            "bottom_span_px": 0,
        }

    expected_height_px = float(fy) * float(nominal_height_m) / float(surface_depth_m)
    if expected_height_px <= 1.0:
        return {
            "partial": False,
            "case": "full_or_unknown",
            "visible_ratio_estimate": 1.0,
            "expected_height_px": expected_height_px,
            "top_span_px": 0,
            "bottom_span_px": 0,
        }

    visible_ratio = float(bbox_height_px) / float(expected_height_px)
    result = {
        "partial": bool(visible_ratio < float(partial_ratio_threshold)),
        "case": "full_or_unknown",
        "visible_ratio_estimate": visible_ratio,
        "expected_height_px": expected_height_px,
        "top_span_px": 0,
        "bottom_span_px": 0,
    }
    if not result["partial"]:
        return result

    band_h = max(6, bbox_height_px // 5)
    top_span = band_horizontal_span(
        depth_frame,
        x1,
        y1,
        x2,
        min(int(y2), int(y1) + band_h),
        reference_depth_m=surface_depth_m,
    )
    bottom_span = band_horizontal_span(
        depth_frame,
        x1,
        max(int(y1), int(y2) - band_h),
        x2,
        y2,
        reference_depth_m=surface_depth_m,
    )
    result["top_span_px"] = int(top_span)
    result["bottom_span_px"] = int(bottom_span)

    if top_span > 0 and bottom_span > 0:
        if float(top_span) <= float(bottom_span) * 0.82:
            result["case"] = "upper_half"
        elif float(bottom_span) <= float(top_span) * 0.82:
            result["case"] = "lower_half"

    return result


def choose_bottle_grasp_depth(
    depth_frame,
    x1,
    y1,
    x2,
    y2,
    *,
    depth_intrin,
    nominal_height_m,
    partial_ratio_threshold,
    partial_grasp_offset_m,
    edge_contact_px,
):
    surface_depth_m, center_x, center_y = robust_bbox_depth(depth_frame, x1, y1, x2, y2)
    visibility = infer_partial_bottle_visibility(
        depth_frame,
        x1,
        y1,
        x2,
        y2,
        surface_depth_m=surface_depth_m,
        fy=float(depth_intrin.fy),
        image_height=depth_frame.get_height(),
        nominal_height_m=nominal_height_m,
        partial_ratio_threshold=partial_ratio_threshold,
        edge_contact_px=edge_contact_px,
    )
    info = {
        "grasp_strategy": "bbox_center",
        "partial_visibility": bool(visibility["partial"]),
        "visibility_case": str(visibility["case"]),
        "visible_ratio_estimate": float(visibility["visible_ratio_estimate"]),
        "expected_height_px": float(visibility["expected_height_px"]),
        "top_span_px": int(visibility["top_span_px"]),
        "bottom_span_px": int(visibility["bottom_span_px"]),
    }

    if (
        surface_depth_m <= 0.0
        or not visibility["partial"]
        or visibility["case"] not in ("upper_half", "lower_half")
    ):
        return surface_depth_m, center_x, center_y, info

    offset_px = int(
        round(float(depth_intrin.fy) * float(partial_grasp_offset_m) / float(surface_depth_m))
    )
    offset_px = max(2, offset_px)
    target_x = int((int(x1) + int(x2)) * 0.5)
    if visibility["case"] == "upper_half":
        target_y = min(int(y2) - 2, int(y1) + offset_px)
        info["grasp_strategy"] = "partial_upper_from_top"
    else:
        target_y = max(int(y1) + 2, int(y2) - offset_px)
        info["grasp_strategy"] = "partial_lower_from_bottom"

    target_depth_m, grasp_x, grasp_y = robust_depth_near_pixel(
        depth_frame,
        target_x,
        target_y,
        bbox=(x1, y1, x2, y2),
        reference_depth_m=surface_depth_m,
        reference_tolerance_m=0.12,
    )
    if target_depth_m <= 0.0:
        info["grasp_strategy"] = "bbox_center_fallback"
        return surface_depth_m, center_x, center_y, info

    return target_depth_m, grasp_x, grasp_y, info


def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class SimpleIMUFusion:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_t = None

    def update(self, accel_xyz, gyro_xyz):
        now = time.monotonic()
        if self.last_t is None:
            self.last_t = now
            return

        dt = now - self.last_t
        self.last_t = now
        if dt <= 0.0 or dt > 0.2:
            return

        ax, ay, az = accel_xyz
        gx, gy, gz = gyro_xyz

        roll_gyro = self.roll + gx * dt
        pitch_gyro = self.pitch + gy * dt
        yaw_gyro = self.yaw + gz * dt

        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        self.roll = self.alpha * roll_gyro + (1.0 - self.alpha) * roll_acc
        self.pitch = self.alpha * pitch_gyro + (1.0 - self.alpha) * pitch_acc
        self.yaw = yaw_gyro


class CameraVision(Node):
    def __init__(self):
        super().__init__("camera_vision")
        self.get_logger().info("Starting end-effector RealSense feedback node")
        numeric_tuning_param = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("model_path", "yoloe-11s-seg.pt")
        self.declare_parameter("use_yoloe", True)
        self.declare_parameter(
            "prompt_classes",
            (
                "green bottle,green tea bottle,orange bottle,"
                "clear bottle,"
                "coca cola can,apple,orange,lemon,"
                "yellow lays potato chips bag,yellow potato chips bag,"
                "side of yellow potato chips bag,yellow chips bag side view,"
                "yellow snack bag"
            ),
        )
        self.declare_parameter(
            "published_class_names",
            (
                "green tea,green tea,roasted tea,"
                "water,"
                "can,apple,orange,lemon,"
                "bag of chips,bag of chips,bag of chips,bag of chips,bag of chips"
            ),
        )
        self.declare_parameter("conf", 0.4)
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("show_live_window", False)
        self.declare_parameter("use_imu", False)
        self.declare_parameter("depth_width", 848)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 30)
        self.declare_parameter("color_width", 1280)
        self.declare_parameter("color_height", 720)
        self.declare_parameter("color_fps", 30)
        self.declare_parameter("color_enable_auto_exposure", False)
        self.declare_parameter("color_exposure", 300.0, numeric_tuning_param)
        self.declare_parameter("color_gain", 24.0, numeric_tuning_param)
        self.declare_parameter("color_brightness", 0.0, numeric_tuning_param)
        self.declare_parameter("color_contrast", 50.0, numeric_tuning_param)
        self.declare_parameter("color_saturation", 64.0, numeric_tuning_param)
        self.declare_parameter("color_enable_auto_white_balance", True)
        self.declare_parameter("color_white_balance", 4600.0, numeric_tuning_param)
        self.declare_parameter("parent_frame", "ee_link")
        self.declare_parameter("camera_mount_frame", "camera_mount_frame")
        self.declare_parameter("camera_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("object_frame_prefix", "object")
        # Default to the current corrected eye-in-hand runtime TF (correct4).
        # These values represent ee_gripper_link -> camera_mount_frame so the existing
        # camera_mount_frame -> camera_color_optical_frame optical-frame rotation still
        # reconstructs the calibrated optical-frame transform.
        self.declare_parameter("mount_xyz", "-0.0462321,0.0292076,0.0590980")
        self.declare_parameter("mount_rpy_deg", "1.1719,0.6432,1.4275")
        self.declare_parameter("optical_frame_rpy_deg", "-90.0,0.0,-90.0")
        self.declare_parameter("grasp_depth_offset_m", 0.02)
        self.declare_parameter("partial_bottle_grasp_enabled", True)
        self.declare_parameter("bottle_nominal_height_m", 0.24)
        self.declare_parameter("bottle_partial_visibility_ratio", 0.70)
        self.declare_parameter("bottle_partial_grasp_offset_m", 0.10)
        self.declare_parameter("bottle_edge_contact_px", 6)
        self.declare_parameter("max_objects_tf", 5)
        self.declare_parameter("status_log_period_sec", 2.0)
        self.declare_parameter("live_window_name", "camera_vision_live")

        requested_model_path = str(self.get_parameter("model_path").value).strip()
        self.use_yoloe = bool(self.get_parameter("use_yoloe").value)
        self.prompt_classes = self._parse_class_list(
            str(self.get_parameter("prompt_classes").value)
        )
        self.published_class_names = self._parse_class_list(
            str(self.get_parameter("published_class_names").value)
        )
        self.prompt_name_to_label = {}
        self.conf_thres = float(self.get_parameter("conf").value)
        use_cuda = bool(self.get_parameter("use_cuda").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)
        self.show_live_window = bool(self.get_parameter("show_live_window").value)
        self.use_imu = bool(self.get_parameter("use_imu").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fps = int(self.get_parameter("depth_fps").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.color_fps = int(self.get_parameter("color_fps").value)
        self.color_enable_auto_exposure = bool(
            self.get_parameter("color_enable_auto_exposure").value
        )
        self.color_exposure = float(self.get_parameter("color_exposure").value)
        self.color_gain = float(self.get_parameter("color_gain").value)
        self.color_brightness = float(self.get_parameter("color_brightness").value)
        self.color_contrast = float(self.get_parameter("color_contrast").value)
        self.color_saturation = float(self.get_parameter("color_saturation").value)
        self.color_enable_auto_white_balance = bool(
            self.get_parameter("color_enable_auto_white_balance").value
        )
        self.color_white_balance = float(
            self.get_parameter("color_white_balance").value
        )
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.camera_mount_frame = str(self.get_parameter("camera_mount_frame").value)
        self.camera_optical_frame = str(self.get_parameter("camera_optical_frame").value)
        self.object_frame_prefix = str(self.get_parameter("object_frame_prefix").value)
        self.grasp_depth_offset_m = float(self.get_parameter("grasp_depth_offset_m").value)
        self.partial_bottle_grasp_enabled = bool(
            self.get_parameter("partial_bottle_grasp_enabled").value
        )
        self.bottle_nominal_height_m = float(
            self.get_parameter("bottle_nominal_height_m").value
        )
        self.bottle_partial_visibility_ratio = float(
            self.get_parameter("bottle_partial_visibility_ratio").value
        )
        self.bottle_partial_grasp_offset_m = float(
            self.get_parameter("bottle_partial_grasp_offset_m").value
        )
        self.bottle_edge_contact_px = int(
            self.get_parameter("bottle_edge_contact_px").value
        )
        self.max_objects_tf = int(self.get_parameter("max_objects_tf").value)
        self.status_log_period_sec = float(self.get_parameter("status_log_period_sec").value)
        self.live_window_name = str(self.get_parameter("live_window_name").value)
        self.weights_dir = self._configure_ultralytics_weights_dir()
        self.model_path = self._resolve_model_path(requested_model_path)

        self.mount_xyz = self._parse_vec3(str(self.get_parameter("mount_xyz").value))
        roll_deg, pitch_deg, yaw_deg = self._parse_vec3(
            str(self.get_parameter("mount_rpy_deg").value)
        )
        self.mount_quat = euler_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        )
        optical_roll_deg, optical_pitch_deg, optical_yaw_deg = self._parse_vec3(
            str(self.get_parameter("optical_frame_rpy_deg").value)
        )
        self.optical_quat = euler_to_quat(
            math.radians(optical_roll_deg),
            math.radians(optical_pitch_deg),
            math.radians(optical_yaw_deg),
        )

        self.pub_detections = self.create_publisher(String, "detections_json", 10)
        self.pub_point = self.create_publisher(PointStamped, "detection_point", 10)
        if self.publish_image:
            self.pub_image = self.create_publisher(Image, "camera/color/image_raw", 10)
            self.bridge = CvBridge()
        self.live_window_enabled = self.show_live_window
        self.live_window_created = False
        self.latest_fps = 0.0
        if self.live_window_enabled and not os.environ.get("DISPLAY"):
            self.get_logger().warn(
                "show_live_window was requested but DISPLAY is not set. "
                "Disabling the live preview window."
            )
            self.live_window_enabled = False
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            f"Camera TF: parent={self.parent_frame} "
            f"mount={self.camera_mount_frame} optical={self.camera_optical_frame}"
        )
        self.get_logger().info(f"Ultralytics weights dir: {self.weights_dir}")

        self._initialize_detection_model(requested_model_path)
        self.device = 0 if (use_cuda and torch.cuda.is_available()) else "cpu"
        if self.device != "cpu":
            self.model.to("cuda")
        self.get_logger().info(f"Using device: {self.device}")

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(
            rs.stream.depth,
            self.depth_width,
            self.depth_height,
            rs.format.z16,
            self.depth_fps,
        )
        cfg.enable_stream(
            rs.stream.color,
            self.color_width,
            self.color_height,
            rs.format.bgr8,
            self.color_fps,
        )
        if self.use_imu:
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        try:
            self.pipeline.start(cfg)
        except RuntimeError as exc:
            self.get_logger().warn(
                f"Primary RealSense profile failed ({exc}). "
                "Falling back to 640x480@30 without IMU."
            )
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(cfg)
            self.use_imu = False
        self._configure_color_sensor()
        self.align_to_color = rs.align(rs.stream.color)

        self.imu_fusion = SimpleIMUFusion(alpha=0.98)
        self.latest_accel = None
        self.latest_gyro = None
        self.fps_hist = deque(maxlen=20)
        self.last_t = time.time()
        self.last_status_log_time = 0.0

        self.timer = self.create_timer(0.02, self.timer_cb)

    @staticmethod
    def _parse_vec3(text):
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Expected 3 comma-separated values, got: {text}")
        return float(parts[0]), float(parts[1]), float(parts[2])

    @staticmethod
    def _parse_class_list(text: str) -> list[str]:
        return [part.strip().lower() for part in text.split(",") if part.strip()]

    @staticmethod
    def _configure_ultralytics_weights_dir() -> Path:
        weights_dir = Path(str(SETTINGS["weights_dir"])).expanduser()
        if not weights_dir.is_absolute():
            weights_dir = Path.home() / ".cache" / "grocerybot_ultralytics" / "weights"
            SETTINGS["weights_dir"] = str(weights_dir)
        weights_dir.mkdir(parents=True, exist_ok=True)
        return weights_dir

    def _resolve_model_path(self, requested_model_path: str) -> str:
        if not requested_model_path:
            return requested_model_path

        requested = Path(requested_model_path).expanduser()
        if requested.is_absolute() and requested.exists():
            return str(requested)
        if requested.exists():
            return str(requested.resolve())

        package_candidate = Path(__file__).resolve().parent / requested.name
        if package_candidate.exists():
            return str(package_candidate)

        if not requested.is_absolute():
            return str(self.weights_dir / requested.name)
        return str(requested)

    def _prepare_yoloe_prompt_assets(self) -> None:
        if not self.prompt_classes:
            return

        clip_asset = self.weights_dir / "mobileclip_blt.ts"
        if clip_asset.exists():
            return

        self.get_logger().info(
            "Preparing YOLOE text-prompt asset mobileclip_blt.ts. "
            "The first run may download about 572MB."
        )
        attempt_download_asset(str(clip_asset))

    def _initialize_detection_model(self, requested_model_path: str) -> None:
        requested_name = Path(requested_model_path).name.lower()
        use_yoloe = self.use_yoloe or "yoloe" in requested_name

        if use_yoloe:
            if self.published_class_names:
                if len(self.published_class_names) != len(self.prompt_classes):
                    raise RuntimeError(
                        "published_class_names must have the same number of entries as "
                        "prompt_classes when YOLOE is enabled."
                    )
                self.prompt_name_to_label = {
                    prompt: label
                    for prompt, label in zip(self.prompt_classes, self.published_class_names)
                }
            else:
                self.prompt_name_to_label = {
                    prompt: prompt
                    for prompt in self.prompt_classes
                }

            self._prepare_yoloe_prompt_assets()
            self.get_logger().info(f"Loading YOLOE model: {self.model_path}")
            self.model = YOLOE(self.model_path, verbose=False)
            if self.prompt_classes:
                self.get_logger().info(
                    "Using YOLOE prompt classes: " + ", ".join(self.prompt_classes)
                )
                if self.prompt_name_to_label:
                    self.get_logger().info(
                        "Publishing YOLOE classes as: "
                        + ", ".join(
                            f"{prompt}->{label}"
                            for prompt, label in self.prompt_name_to_label.items()
                        )
                    )
                try:
                    self.model.set_classes(self.prompt_classes)
                except Exception as exc:
                    raise RuntimeError(
                        "Failed to initialize YOLOE prompt classes. "
                        "Ensure CLIP and the MobileCLIP text encoder are available."
                    ) from exc
            else:
                self.get_logger().warn(
                    "YOLOE is enabled but prompt_classes is empty; detection labels will not "
                    "match your grocery products."
                )
            return

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

    def _configure_color_sensor(self) -> None:
        try:
            device = self.pipeline.get_active_profile().get_device()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Unable to access RealSense device profile: {exc}")
            return

        color_sensor = None
        try:
            for sensor in device.query_sensors():
                sensor_name = ""
                try:
                    sensor_name = str(sensor.get_info(rs.camera_info.name))
                except Exception:  # noqa: BLE001
                    sensor_name = ""
                if "rgb" in sensor_name.lower() or "color" in sensor_name.lower():
                    color_sensor = sensor
                    break
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Unable to enumerate RealSense sensors: {exc}")
            return

        if color_sensor is None:
            self.get_logger().warn("No RealSense color sensor found for manual tuning.")
            return

        self._set_sensor_option(
            color_sensor,
            rs.option.enable_auto_exposure,
            1.0 if self.color_enable_auto_exposure else 0.0,
            "enable_auto_exposure",
        )
        if not self.color_enable_auto_exposure:
            self._set_sensor_option(
                color_sensor,
                rs.option.exposure,
                self.color_exposure,
                "exposure",
            )
            self._set_sensor_option(
                color_sensor,
                rs.option.gain,
                self.color_gain,
                "gain",
            )

        self._set_sensor_option(
            color_sensor,
            rs.option.brightness,
            self.color_brightness,
            "brightness",
        )
        self._set_sensor_option(
            color_sensor,
            rs.option.contrast,
            self.color_contrast,
            "contrast",
        )
        self._set_sensor_option(
            color_sensor,
            rs.option.saturation,
            self.color_saturation,
            "saturation",
        )
        self._set_sensor_option(
            color_sensor,
            rs.option.enable_auto_white_balance,
            1.0 if self.color_enable_auto_white_balance else 0.0,
            "enable_auto_white_balance",
        )
        if not self.color_enable_auto_white_balance:
            self._set_sensor_option(
                color_sensor,
                rs.option.white_balance,
                self.color_white_balance,
                "white_balance",
            )

        self.get_logger().info(
            "RealSense color tuning: "
            f"auto_exposure={self.color_enable_auto_exposure} "
            f"exposure={self.color_exposure:.1f} "
            f"gain={self.color_gain:.1f} "
            f"brightness={self.color_brightness:.1f} "
            f"contrast={self.color_contrast:.1f} "
            f"saturation={self.color_saturation:.1f} "
            f"auto_white_balance={self.color_enable_auto_white_balance} "
            f"white_balance={self.color_white_balance:.1f}"
        )

    def _set_sensor_option(self, sensor, option, value: float, option_name: str) -> None:
        try:
            if not sensor.supports(option):
                return
            option_range = sensor.get_option_range(option)
            clamped = min(max(float(value), float(option_range.min)), float(option_range.max))
            sensor.set_option(option, clamped)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Unable to set RealSense color option {option_name}: {exc}"
            )

    def _update_imu(self, frames):
        for f in frames:
            st = f.get_profile().stream_type()
            if st == rs.stream.accel:
                m = f.as_motion_frame().get_motion_data()
                self.latest_accel = (float(m.x), float(m.y), float(m.z))
            elif st == rs.stream.gyro:
                m = f.as_motion_frame().get_motion_data()
                self.latest_gyro = (float(m.x), float(m.y), float(m.z))

        if self.latest_accel is not None and self.latest_gyro is not None:
            self.imu_fusion.update(self.latest_accel, self.latest_gyro)

    def _broadcast_camera_frames_tf(self, stamp):
        imu_quat = euler_to_quat(
            self.imu_fusion.roll, self.imu_fusion.pitch, self.imu_fusion.yaw
        )
        mount_quat = quat_multiply(self.mount_quat, imu_quat)

        mount_tf = TransformStamped()
        mount_tf.header.stamp = stamp
        mount_tf.header.frame_id = self.parent_frame
        mount_tf.child_frame_id = self.camera_mount_frame
        mount_tf.transform.translation.x = float(self.mount_xyz[0])
        mount_tf.transform.translation.y = float(self.mount_xyz[1])
        mount_tf.transform.translation.z = float(self.mount_xyz[2])
        mount_tf.transform.rotation.x = float(mount_quat[0])
        mount_tf.transform.rotation.y = float(mount_quat[1])
        mount_tf.transform.rotation.z = float(mount_quat[2])
        mount_tf.transform.rotation.w = float(mount_quat[3])

        optical_tf = TransformStamped()
        optical_tf.header.stamp = stamp
        optical_tf.header.frame_id = self.camera_mount_frame
        optical_tf.child_frame_id = self.camera_optical_frame
        optical_tf.transform.translation.x = 0.0
        optical_tf.transform.translation.y = 0.0
        optical_tf.transform.translation.z = 0.0
        optical_tf.transform.rotation.x = float(self.optical_quat[0])
        optical_tf.transform.rotation.y = float(self.optical_quat[1])
        optical_tf.transform.rotation.z = float(self.optical_quat[2])
        optical_tf.transform.rotation.w = float(self.optical_quat[3])

        self.tf_broadcaster.sendTransform(mount_tf)
        self.tf_broadcaster.sendTransform(optical_tf)

    def _broadcast_object_tf(self, stamp, detections):
        for i, det in enumerate(detections[: self.max_objects_tf]):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.camera_optical_frame
            t.child_frame_id = f"{self.object_frame_prefix}_{i}"
            point_camera_optical_m = det["point_camera_optical_m"]
            t.transform.translation.x = float(point_camera_optical_m[0])
            t.transform.translation.y = float(point_camera_optical_m[1])
            t.transform.translation.z = float(point_camera_optical_m[2])
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

    def _maybe_log_status(self, detections):
        if self.status_log_period_sec <= 0.0:
            return
        now = time.time()
        if (now - self.last_status_log_time) < self.status_log_period_sec:
            return
        self.last_status_log_time = now

        if detections:
            first = detections[0]
            point = first["point_camera_optical_m"]
            class_names = list(dict.fromkeys(str(det["class_name"]) for det in detections))
            self.get_logger().info(
                "camera_vision status: "
                f"detections={len(detections)} "
                f"first_class={first['class_name']} "
                f"conf={float(first['confidence']):.2f} "
                f"depth_m={float(first['distance_m']):.3f} "
                f"optical_xyz=({float(point[0]):.3f}, {float(point[1]):.3f}, {float(point[2]):.3f})"
            )
            self.get_logger().info(
                "camera_vision grasp: "
                f"strategy={first.get('grasp_strategy', 'bbox_center')} "
                f"case={first.get('visibility_case', 'full_or_unknown')} "
                f"ratio={float(first.get('visible_ratio_estimate', 1.0)):.2f} "
                f"grasp_px={tuple(first.get('grasp_px', [0, 0]))}"
            )
            self.get_logger().info(
                "camera_vision detected_classes: "
                + ", ".join(class_names)
            )
        else:
            self.get_logger().info("camera_vision status: frames OK, detections=0")

    @staticmethod
    def _draw_crosshair(image, x, y, color, radius=10, thickness=2):
        cv2.circle(image, (x, y), radius, color, thickness)
        cv2.line(image, (x - radius - 4, y), (x + radius + 4, y), color, thickness)
        cv2.line(image, (x, y - radius - 4), (x, y + radius + 4), color, thickness)

    @staticmethod
    def _draw_text_block(image, lines, origin, fg_color=(255, 255, 255), bg_color=(24, 24, 24)):
        if not lines:
            return

        x, y = origin
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        padding = 6
        line_gap = 6

        sizes = [cv2.getTextSize(line, font, font_scale, thickness)[0] for line in lines]
        width = max(size[0] for size in sizes) + padding * 2
        line_height = max(size[1] for size in sizes)
        height = len(lines) * line_height + (len(lines) - 1) * line_gap + padding * 2

        top_left = (x, y)
        bottom_right = (x + width, y + height)
        cv2.rectangle(image, top_left, bottom_right, bg_color, -1)
        cv2.rectangle(image, top_left, bottom_right, (80, 80, 80), 1)

        baseline_y = y + padding + line_height
        for line in lines:
            cv2.putText(
                image,
                line,
                (x + padding, baseline_y),
                font,
                font_scale,
                fg_color,
                thickness,
                cv2.LINE_AA,
            )
            baseline_y += line_height + line_gap

    def _annotate_preview(self, image, detections):
        annotated = image.copy()
        panel_lines = [
            "camera_vision live",
            f"fps={self.latest_fps:.1f}",
            f"detections={len(detections)}",
            f"frame={self.camera_optical_frame}",
            "grasp target = partial-aware bottle heuristic",
            f"depth offset = {self.grasp_depth_offset_m:.3f}m",
        ]
        self._draw_text_block(annotated, panel_lines, (12, 12))

        for index, det in enumerate(detections):
            x1, y1, x2, y2 = det["bbox"]
            cx, cy = det["grasp_px"]
            X, Y, Z = det["point_camera_optical_m"]
            depth_m = float(det["distance_m"])
            conf = float(det["confidence"])
            label_color = (0, 220, 0)
            grasp_color = (0, 165, 255)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), label_color, 2)
            self._draw_crosshair(annotated, cx, cy, grasp_color)

            label_lines = [
                f"#{index} {det['class_name']} conf={conf:.2f}",
                f"depth={depth_m:.3f}m px=({cx},{cy})",
                f"opt_xyz=({float(X):.3f}, {float(Y):.3f}, {float(Z):.3f})",
                f"{det.get('grasp_strategy', 'bbox_center')} {det.get('visibility_case', 'full_or_unknown')}",
            ]
            text_y = max(12, y1 - 92)
            self._draw_text_block(
                annotated,
                label_lines,
                (x1, text_y),
                fg_color=(255, 255, 255),
                bg_color=(24, 70, 24),
            )

        return annotated

    def _show_live_preview(self, image):
        if not self.live_window_enabled:
            return

        try:
            if not self.live_window_created:
                cv2.namedWindow(self.live_window_name, cv2.WINDOW_NORMAL)
                self.live_window_created = True

            cv2.imshow(self.live_window_name, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                self.get_logger().info(
                    "Closing camera_vision live preview window; preview can be re-enabled on the next launch."
                )
                cv2.destroyWindow(self.live_window_name)
                self.live_window_created = False
                self.live_window_enabled = False
        except cv2.error as exc:
            self.get_logger().warn(
                f"Disabling live preview window after OpenCV error: {exc}"
            )
            self.live_window_enabled = False
            self.live_window_created = False

    def timer_cb(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=3000)
        except Exception as e:
            self.get_logger().error(f"RealSense capture error: {e}")
            return

        if self.use_imu:
            self._update_imu(frames)
        aligned = self.align_to_color.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            return

        color_img = np.asanyarray(color.get_data())
        depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()
        results = self.model.predict(
            source=color_img,
            conf=self.conf_thres,
            iou=0.45,
            device=0 if self.device != "cpu" else "cpu",
            verbose=False,
        )

        detections_out = []
        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                model_name = str(names.get(cls_id, str(cls_id))).strip()
                name = self.prompt_name_to_label.get(
                    model_name.lower(),
                    model_name,
                )
                bbox_center_x = int((x1 + x2) * 0.5)
                bbox_center_y = int((y1 + y2) * 0.5)

                class_name_lower = str(name).strip().lower()
                is_bottle = "bottle" in class_name_lower
                if self.partial_bottle_grasp_enabled and is_bottle:
                    surface_dist_m, cx, cy, grasp_info = choose_bottle_grasp_depth(
                        depth,
                        x1,
                        y1,
                        x2,
                        y2,
                        depth_intrin=depth_intrin,
                        nominal_height_m=self.bottle_nominal_height_m,
                        partial_ratio_threshold=self.bottle_partial_visibility_ratio,
                        partial_grasp_offset_m=self.bottle_partial_grasp_offset_m,
                        edge_contact_px=self.bottle_edge_contact_px,
                    )
                else:
                    surface_dist_m, cx, cy = robust_bbox_depth(depth, x1, y1, x2, y2)
                    grasp_info = {
                        "grasp_strategy": "bbox_center",
                        "partial_visibility": False,
                        "visibility_case": "full_or_unknown",
                        "visible_ratio_estimate": 1.0,
                        "expected_height_px": 0.0,
                        "top_span_px": 0,
                        "bottom_span_px": 0,
                    }
                dist_m = surface_dist_m
                if dist_m > 0.0:
                    dist_m = max(0.0, dist_m + self.grasp_depth_offset_m)
                    # RealSense deprojection returns a 3D point in the camera optical frame:
                    # x right, y down, z forward.
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(
                        depth_intrin, [float(cx), float(cy)], float(dist_m)
                    )
                else:
                    X = Y = Z = 0.0

                point_camera_optical_m = [float(X), float(Y), float(Z)]

                det = {
                    "class_id": cls_id,
                    "class_name": name,
                    "model_class_name": model_name,
                    "confidence": conf,
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center_px": [bbox_center_x, bbox_center_y],
                    "distance_m": float(dist_m),
                    "surface_distance_m": float(surface_dist_m),
                    "point_camera_optical_m": point_camera_optical_m,
                    "grasp_px": [cx, cy],
                    "grasp_strategy": str(grasp_info["grasp_strategy"]),
                    "partial_visibility": bool(grasp_info["partial_visibility"]),
                    "visibility_case": str(grasp_info["visibility_case"]),
                    "visible_ratio_estimate": float(grasp_info["visible_ratio_estimate"]),
                    "expected_height_px": float(grasp_info["expected_height_px"]),
                    "top_span_px": int(grasp_info["top_span_px"]),
                    "bottom_span_px": int(grasp_info["bottom_span_px"]),
                }
                detections_out.append(det)

                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = self.camera_optical_frame
                pt_msg.point.x = float(X)
                pt_msg.point.y = float(Y)
                pt_msg.point.z = float(Z)
                self.pub_point.publish(pt_msg)

        stamp = self.get_clock().now().to_msg()
        self._broadcast_camera_frames_tf(stamp)
        if detections_out:
            self._broadcast_object_tf(stamp, detections_out)
        self._maybe_log_status(detections_out)

        payload = {
            "timestamp": time.time(),
            "camera_optical_frame": self.camera_optical_frame,
            "camera_mount_frame": self.camera_mount_frame,
            "imu_rpy_rad": [
                float(self.imu_fusion.roll),
                float(self.imu_fusion.pitch),
                float(self.imu_fusion.yaw),
            ],
            "detections": detections_out,
        }
        self.pub_detections.publish(String(data=json.dumps(payload)))

        preview_img = None
        if self.publish_image or self.live_window_enabled:
            preview_img = self._annotate_preview(color_img, detections_out)

        if self.publish_image:
            ros_img = self.bridge.cv2_to_imgmsg(preview_img, encoding="bgr8")
            ros_img.header.stamp = stamp
            ros_img.header.frame_id = self.camera_optical_frame
            self.pub_image.publish(ros_img)

        if preview_img is not None:
            self._show_live_preview(preview_img)

        now = time.time()
        fps = 1.0 / max(1e-6, now - self.last_t)
        self.last_t = now
        self.fps_hist.append(fps)
        self.latest_fps = sum(self.fps_hist) / len(self.fps_hist)
        if len(self.fps_hist) == self.fps_hist.maxlen:
            self.get_logger().debug(
                f"FPS={self.latest_fps:.1f}, "
                f"detections={len(detections_out)}"
            )

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.live_window_created:
            try:
                cv2.destroyWindow(self.live_window_name)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
