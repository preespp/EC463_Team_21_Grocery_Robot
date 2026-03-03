"""
Grocery Robot Vision System (Updated for Base/Map Coordinates + IMU + Jetson Output)
===================================================================================
Features:
  1) YOLO product detection + 3D position + size estimation (for robotic arm grasping)
  2) ZXing barcode scanning + robot localization (landmark-based)
  3) IMU export (RealSense D435i gyro+accel) + yaw alignment to MAP via barcode
  4) Coordinate transforms:
        - pos_cam : 3D position in CAMERA frame (meters)
        - pos_base: 3D position in BASE(chassis) frame (meters) using camera extrinsic (offset + rpy)
        - pos_map : 3D position in MAP/world frame (meters) using base map pose + IMU yaw (+ offset)
  5) Optional UDP JSON streaming to Jetson

Usage:
  python realsense_grocery_robot.py --model yolov8n.pt --conf 0.5 \
      --use_imu \
      --cam_offset 0.12,0.00,0.35 \
      --cam_rpy 0,0,0 \
      --udp 192.168.1.88:5005

Coordinate frames (convention):
  - CAMERA frame (RealSense deprojection): X right, Y down, Z forward
  - BASE frame (ROS-like): X forward, Y left, Z up
  - MAP frame: X east/forward, Y north/left, Z up (2D heading theta around +Z)

Notes:
  - IMU yaw will drift over time (no magnetometer). We correct it by aligning to barcode landmarks:
      yaw_offset_deg = map_theta_deg - imu_yaw_deg_at_detection
    Then heading used for map projection:
      yaw_map_deg = imu_yaw_deg + yaw_offset_deg
"""

import argparse
import time
import json
import math
import socket
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
import zxingcpp
from ultralytics import YOLO


# ============ Configuration ============

# Barcode -> Map coordinates (x, y, theta)
# Units: meters / degrees
BARCODE_MAP = {
    "Location_1": (3.00, 4.00, 0.0),
    "Location_2": (1.50, 1.20, 0.0),
    "Location_3": (0.50, 2.40, 90.0),
    # Add more as needed...
}


# ============ Utility Functions ============

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def parse_host_port(s: str):
    # "ip:port"
    if not s:
        return None
    if ":" not in s:
        raise ValueError("UDP must be in form ip:port, e.g. 192.168.1.10:5005")
    host, port = s.split(":", 1)
    return host.strip(), int(port.strip())


def parse_vec3_csv(s: str):
    # "x,y,z"
    parts = [p.strip() for p in s.split(",")]
    if len(parts) != 3:
        raise ValueError("Expected 3 comma-separated values like '0.1,0,0.3'")
    return float(parts[0]), float(parts[1]), float(parts[2])


def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi


def rot_x(roll):
    cr, sr = math.cos(roll), math.sin(roll)
    return np.array([[1, 0, 0],
                     [0, cr, -sr],
                     [0, sr, cr]], dtype=np.float64)


def rot_y(pitch):
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.array([[cp, 0, sp],
                     [0, 1, 0],
                     [-sp, 0, cp]], dtype=np.float64)


def rot_z(yaw):
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy, -sy, 0],
                     [sy,  cy, 0],
                     [0,    0, 1]], dtype=np.float64)


def rpy_to_R(roll, pitch, yaw):
    # ZYX (yaw->pitch->roll) is common in robotics
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def median_depth(depth_frame, x, y, k=5):
    """
    Get median depth (in meters) from a kxk region around (x, y), ignoring zero values.
    More robust than single-point depth.
    """
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k // 2), max(0, y - k // 2)
    x1, y1 = min(w, x + k // 2 + 1), min(h, y + k // 2 + 1)
    vals = []
    for yy in range(y0, y1):
        for xx in range(x0, x1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0


def get_grasp_depth(depth_frame, x1, y1, x2, y2):
    """
    Get the closest (minimum) valid depth in the bounding box.
    Better for grasping than center point depth.
    Returns (depth_m, best_x, best_y) - the depth and pixel location of closest point.
    """
    h, w = depth_frame.get_height(), depth_frame.get_width()

    # Shrink bbox slightly to avoid edge noise
    margin_x = (x2 - x1) // 4
    margin_y = (y2 - y1) // 4
    bx1, by1 = x1 + margin_x, y1 + margin_y
    bx2, by2 = x2 - margin_x, y2 - margin_y

    # Clamp to image bounds
    bx1, by1 = max(0, bx1), max(0, by1)
    bx2, by2 = min(w, bx2), min(h, by2)

    min_depth = float('inf')
    best_x, best_y = (bx1 + bx2) // 2, (by1 + by2) // 2

    for yy in range(by1, by2, 2):
        for xx in range(bx1, bx2, 2):
            d = depth_frame.get_distance(xx, yy)
            if 0.1 < d < 3.0 and d < min_depth:
                min_depth = d
                best_x, best_y = xx, yy

    if min_depth == float('inf'):
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        return median_depth(depth_frame, cx, cy, k=5), cx, cy

    return min_depth, best_x, best_y


def get_barcode_corners(position):
    pts = []
    for attr in ("top_left", "top_right", "bottom_right", "bottom_left"):
        if hasattr(position, attr):
            p = getattr(position, attr)
            if hasattr(p, "x") and hasattr(p, "y"):
                pts.append((int(p.x), int(p.y)))
    return pts if pts else None


def pixel_to_3d(depth_intrin, cx, cy, dist_m):
    """
    Convert pixel coordinates + depth to a 3D point in camera frame.
    Returns (X, Y, Z) in meters in CAMERA frame (X right, Y down, Z forward).
    """
    X, Y, Z = rs.rs2_deproject_pixel_to_point(
        depth_intrin, [float(cx), float(cy)], float(dist_m)
    )
    return float(X), float(Y), float(Z)


def cam_to_base_axes_only(p_cam):
    """
    Convert CAMERA axes (X right, Y down, Z forward)
    to BASE axes (X forward, Y left, Z up) without extra rotation.
      base_x = cam_z
      base_y = -cam_x
      base_z = -cam_y
    """
    x, y, z = p_cam
    return np.array([z, -x, -y], dtype=np.float64)


# ============ IMU Tracker (simple complementary filter) ============

class SimpleIMUTracker:
    """
    Uses accel to stabilize roll/pitch, integrates gyro for yaw.
    Yaw drifts (no magnetometer). We'll align yaw to MAP using barcode landmarks.
    """
    def __init__(self, alpha=0.98):
        self.alpha = float(clamp(alpha, 0.0, 1.0))
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_ts = None

    def update(self, accel_xyz, gyro_xyz, ts_s):
        """
        accel_xyz: (ax, ay, az) in m/s^2 (device frame)
        gyro_xyz : (gx, gy, gz) in rad/s (device frame)
        ts_s     : timestamp seconds
        """
        if self._last_ts is None:
            self._last_ts = ts_s
            return

        dt = ts_s - self._last_ts
        self._last_ts = ts_s
        if dt <= 0 or dt > 0.2:
            return

        ax, ay, az = accel_xyz
        gx, gy, gz = gyro_xyz

        # Integrate gyro
        roll_gyro = self.roll + gx * dt
        pitch_gyro = self.pitch + gy * dt
        yaw_gyro = self.yaw + gz * dt

        # Roll/pitch from accel (gravity)
        # NOTE: This depends on IMU axis alignment; good enough for “导出 + 轻量 offset”
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Complementary filter
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc
        self.yaw = yaw_gyro

    def euler_deg(self):
        return (rad2deg(self.roll), rad2deg(self.pitch), rad2deg(self.yaw))


# ============ Detection Result Classes ============

class DetectedProduct:
    """
    Detected product/item
    """
    def __init__(self, name, confidence, bbox, center_px, depth_m, pos_cam,
                 width_m=0.0, height_m=0.0, grasp_point_px=None,
                 pos_base=None, pos_map=None):
        self.name = name
        self.confidence = confidence
        self.bbox = bbox
        self.center_px = center_px
        self.depth_m = depth_m

        self.pos_cam = pos_cam          # CAMERA frame
        self.pos_base = pos_base        # BASE frame (after offset)
        self.pos_map = pos_map          # MAP frame (absolute) if available

        self.width_m = width_m
        self.height_m = height_m
        self.grasp_point_px = grasp_point_px or center_px

    def __repr__(self):
        pc = self.pos_cam
        pb = self.pos_base if self.pos_base is not None else (0, 0, 0)
        return (f"DetectedProduct(name='{self.name}', conf={self.confidence:.2f}, "
                f"cam=[{pc[0]:.3f},{pc[1]:.3f},{pc[2]:.3f}]m, "
                f"base=[{pb[0]:.3f},{pb[1]:.3f},{pb[2]:.3f}]m, "
                f"size=[{self.width_m*100:.1f}x{self.height_m*100:.1f}]cm)")


class DetectedBarcode:
    """
    Detected barcode
    """
    def __init__(self, text, format_type, corners, center_px, depth_m, pos_cam, map_pose=None):
        self.text = text
        self.format_type = format_type
        self.corners = corners
        self.center_px = center_px
        self.depth_m = depth_m
        self.pos_cam = pos_cam
        self.map_pose = map_pose

    def __repr__(self):
        if self.map_pose:
            return f"DetectedBarcode(text='{self.text}', map_pose={self.map_pose})"
        return f"DetectedBarcode(text='{self.text}', UNREGISTERED)"


# ============ Main Vision Class ============

class GroceryRobotVision:
    """
    Grocery Robot Vision System
    Adds:
      - camera->base transform using --cam_offset and --cam_rpy
      - optional IMU and map projection
      - optional UDP JSON streaming
    """

    def __init__(self,
                 yolo_model_path="yolov8n.pt",
                 conf_threshold=0.5,
                 use_imu=False,
                 cam_offset_base=(0.0, 0.0, 0.0),
                 cam_rpy_deg=(0.0, 0.0, 0.0),
                 udp_target=None):
        self.conf_threshold = conf_threshold
        self.use_imu = use_imu

        # Camera extrinsic: base_T_cam (we transform points from camera -> base)
        self.t_base_cam = np.array(cam_offset_base, dtype=np.float64)  # meters
        roll, pitch, yaw = [deg2rad(v) for v in cam_rpy_deg]
        self.R_base_cam_extra = rpy_to_R(roll, pitch, yaw)

        # RealSense pipeline
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        if self.use_imu:
            # D435i motion streams
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
            cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)

        # IMU tracker
        self.imu = SimpleIMUTracker(alpha=0.98) if self.use_imu else None

        # MAP pose state
        self.base_pose_map = None   # (x, y, theta_deg)
        self.yaw_offset_deg = 0.0   # align imu yaw -> map theta

        # UDP
        self.udp_target = udp_target
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if udp_target else None

        # Initialize YOLO
        print(f"[INFO] Loading YOLO model: {yolo_model_path}")
        self.yolo_model = YOLO(yolo_model_path)

        # FPS
        self.fps_hist = deque(maxlen=20)

        # Warm up
        print("[INFO] Warming up camera...")
        for _ in range(10):
            self.pipe.wait_for_frames(10000)
        print("[INFO] Vision system ready!")

    def _extract_motion(self, frameset):
        """
        Try to extract latest accel+gyro frames from frameset (if enabled).
        Returns (accel_xyz, gyro_xyz, ts_s) or None.
        """
        if not self.use_imu:
            return None

        try:
            accel_frame = frameset.first_or_default(rs.stream.accel)
            gyro_frame = frameset.first_or_default(rs.stream.gyro)
            if not accel_frame or not gyro_frame:
                return None

            a = accel_frame.as_motion_frame().get_motion_data()
            g = gyro_frame.as_motion_frame().get_motion_data()

            # Use RealSense timestamps (ms) -> seconds
            ts_s = float(frameset.get_timestamp()) / 1000.0

            accel_xyz = (float(a.x), float(a.y), float(a.z))
            gyro_xyz = (float(g.x), float(g.y), float(g.z))
            return accel_xyz, gyro_xyz, ts_s
        except Exception:
            return None

    def get_frames(self):
        """
        Get aligned depth and color frames.
        Also updates IMU if enabled.
        """
        frames = self.pipe.wait_for_frames(10000)

        # update IMU
        motion = self._extract_motion(frames)
        if motion and self.imu:
            accel_xyz, gyro_xyz, ts_s = motion
            self.imu.update(accel_xyz, gyro_xyz, ts_s)

        aligned = self.align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        return depth, color

    def camera_pose_base(self):
        """
        The pose of camera origin expressed in BASE frame.
        For now we only output the configured extrinsic (static).
        If later camera is on an arm joint, replace this with FK output.
        """
        # rotation output in degrees (rpy) is whatever you configured
        return {
            "t_base_cam_m": [float(self.t_base_cam[0]), float(self.t_base_cam[1]), float(self.t_base_cam[2])],
        }

    def cam_point_to_base(self, pos_cam):
        """
        pos_cam: (X,Y,Z) in CAMERA frame
        Steps:
          1) convert camera axes -> base axes (canonical mapping)
          2) apply extra rotation (cam_rpy) if you configured
          3) add translation offset (cam_offset)
        """
        p_cam = np.array(pos_cam, dtype=np.float64)
        p_base0 = cam_to_base_axes_only(p_cam)
        p_base = self.R_base_cam_extra @ p_base0 + self.t_base_cam
        return (float(p_base[0]), float(p_base[1]), float(p_base[2]))

    def base_point_to_map(self, pos_base):
        """
        pos_base: (x,y,z) in BASE frame
        Use current base_pose_map and IMU yaw to compute absolute MAP position.
        If base_pose_map is None, returns None.
        """
        if self.base_pose_map is None:
            return None

        bx, by, btheta_deg = self.base_pose_map

        imu_yaw_deg = self.imu.euler_deg()[2] if (self.use_imu and self.imu) else btheta_deg
        yaw_map_deg = imu_yaw_deg + self.yaw_offset_deg

        yaw = deg2rad(yaw_map_deg)
        R = rot_z(yaw)

        p = np.array([pos_base[0], pos_base[1], pos_base[2]], dtype=np.float64)
        p_xy = R @ p  # rotate around Z
        # translate by base map position
        p_xy[0] += bx
        p_xy[1] += by
        # z keep as is
        return (float(p_xy[0]), float(p_xy[1]), float(p_xy[2]), float(yaw_map_deg))

    def detect_products(self, color_img, depth_frame):
        products = []

        depth_intrin = depth_frame.profile.as_video_stream_profile().get_intrinsics()
        fx = depth_intrin.fx
        fy = depth_intrin.fy

        results = self.yolo_model(color_img, conf=self.conf_threshold, iou=0.45, verbose=False)

        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                name = names.get(cls_id, str(cls_id))

                depth_m, grasp_x, grasp_y = get_grasp_depth(depth_frame, x1, y1, x2, y2)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

                if depth_m > 0:
                    pos_cam = pixel_to_3d(depth_intrin, grasp_x, grasp_y, depth_m)
                else:
                    pos_cam = (0.0, 0.0, 0.0)

                # size estimation
                width_px = x2 - x1
                height_px = y2 - y1
                if depth_m > 0 and fx > 0 and fy > 0:
                    width_m = width_px * depth_m / fx
                    height_m = height_px * depth_m / fy
                else:
                    width_m, height_m = 0.0, 0.0

                # NEW: camera -> base
                pos_base = self.cam_point_to_base(pos_cam) if depth_m > 0 else None

                # NEW: base -> map (absolute) if we have landmark pose
                pos_map = self.base_point_to_map(pos_base) if (pos_base is not None) else None

                products.append(DetectedProduct(
                    name=name,
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    center_px=(cx, cy),
                    depth_m=depth_m,
                    pos_cam=pos_cam,
                    width_m=width_m,
                    height_m=height_m,
                    grasp_point_px=(grasp_x, grasp_y),
                    pos_base=pos_base,
                    pos_map=pos_map
                ))

        return products

    def detect_barcodes(self, color_img, depth_frame):
        barcodes = []
        depth_intrin = depth_frame.profile.as_video_stream_profile().get_intrinsics()

        results = zxingcpp.read_barcodes(color_img)

        for r in results:
            corners = get_barcode_corners(r.position)

            if corners:
                cx = int(np.mean([p[0] for p in corners]))
                cy = int(np.mean([p[1] for p in corners]))
            else:
                cx, cy = color_img.shape[1] // 2, color_img.shape[0] // 2

            dist_m = median_depth(depth_frame, cx, cy, k=9)
            pos_cam = pixel_to_3d(depth_intrin, cx, cy, dist_m) if dist_m > 0 else (0.0, 0.0, 0.0)

            map_pose = BARCODE_MAP.get(r.text, None)

            # NEW: if barcode is registered, update base pose + yaw alignment
            if map_pose is not None:
                self.base_pose_map = map_pose  # (x, y, theta_deg)

                if self.use_imu and self.imu:
                    imu_yaw_deg = self.imu.euler_deg()[2]
                    self.yaw_offset_deg = map_pose[2] - imu_yaw_deg

            barcodes.append(DetectedBarcode(
                text=r.text,
                format_type=str(r.format),
                corners=corners,
                center_px=(cx, cy),
                depth_m=dist_m,
                pos_cam=pos_cam,
                map_pose=map_pose
            ))

        return barcodes

    def process_frame(self):
        t0 = time.time()

        depth, color = self.get_frames()
        if not depth or not color:
            return None, [], [], 0, None

        color_img = np.asanyarray(color.get_data())

        products = self.detect_products(color_img, depth)
        barcodes = self.detect_barcodes(color_img, depth)

        fps = 1.0 / max(1e-6, time.time() - t0)
        self.fps_hist.append(fps)

        imu_euler = self.imu.euler_deg() if (self.use_imu and self.imu) else None
        return color_img, products, barcodes, float(np.mean(self.fps_hist)), imu_euler

    def _build_payload(self, products, barcodes, fps, imu_euler):
        """
        Build JSON payload to send to Jetson.
        Includes:
          - base absolute pose (if known)
          - camera pose in base (offset)
          - imu euler
          - product positions: cam/base/map
        """
        ts = datetime.now().isoformat(timespec="milliseconds")

        base_pose = None
        if self.base_pose_map is not None:
            bx, by, btheta = self.base_pose_map
            imu_yaw_deg = imu_euler[2] if imu_euler else btheta
            yaw_map_deg = imu_yaw_deg + self.yaw_offset_deg
            base_pose = {
                "x_m": float(bx),
                "y_m": float(by),
                "theta_deg_landmark": float(btheta),
                "yaw_deg_imu": float(imu_yaw_deg),
                "yaw_offset_deg": float(self.yaw_offset_deg),
                "yaw_deg_used": float(yaw_map_deg),
            }

        payload = {
            "timestamp": ts,
            "fps": float(fps),
            "imu_euler_deg": None if imu_euler is None else {
                "roll": float(imu_euler[0]),
                "pitch": float(imu_euler[1]),
                "yaw": float(imu_euler[2]),
            },
            "base_pose_map": base_pose,
            # “装着相机的关节/位置坐标”（当前用静态外参表达；后续可替换成关节FK实时输出）
            "camera_pose_base": self.camera_pose_base(),
            "products": [],
            "barcodes": [],
        }

        for p in products:
            item = {
                "name": p.name,
                "confidence": float(p.confidence),
                "bbox_px": [int(v) for v in p.bbox],
                "grasp_px": [int(p.grasp_point_px[0]), int(p.grasp_point_px[1])],
                "depth_m": float(p.depth_m),
                "pos_cam_m": [float(p.pos_cam[0]), float(p.pos_cam[1]), float(p.pos_cam[2])],
                "pos_base_m": None if p.pos_base is None else [float(p.pos_base[0]), float(p.pos_base[1]), float(p.pos_base[2])],
                # pos_map also includes yaw used (4th value) for convenience
                "pos_map_m": None if p.pos_map is None else [float(p.pos_map[0]), float(p.pos_map[1]), float(p.pos_map[2])],
                "yaw_map_deg": None if p.pos_map is None else float(p.pos_map[3]),
                "size_m": [float(p.width_m), float(p.height_m)],
            }
            payload["products"].append(item)

        for b in barcodes:
            payload["barcodes"].append({
                "text": b.text,
                "format": b.format_type,
                "depth_m": float(b.depth_m),
                "pos_cam_m": [float(b.pos_cam[0]), float(b.pos_cam[1]), float(b.pos_cam[2])],
                "map_pose": None if b.map_pose is None else {
                    "x_m": float(b.map_pose[0]),
                    "y_m": float(b.map_pose[1]),
                    "theta_deg": float(b.map_pose[2]),
                }
            })

        return payload

    def _send_udp(self, payload):
        if not self.udp_sock or not self.udp_target:
            return
        try:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.udp_sock.sendto(data, self.udp_target)
        except Exception:
            pass

    def draw_results(self, color_img, products, barcodes, fps, imu_euler):
        # Products
        for p in products:
            x1, y1, x2, y2 = p.bbox
            cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            label = f"{p.name} {p.confidence:.2f}"
            if p.depth_m > 0:
                label += f" | {p.depth_m:.2f}m"
            cv2.putText(color_img, label, (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if p.depth_m > 0:
                Xc, Yc, Zc = p.pos_cam
                cv2.putText(color_img, f"CAM: [{Xc:.2f}, {Yc:.2f}, {Zc:.2f}]",
                            (x1, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                if p.pos_base is not None:
                    Xb, Yb, Zb = p.pos_base
                    cv2.putText(color_img, f"BASE: [{Xb:.2f}, {Yb:.2f}, {Zb:.2f}]",
                                (x1, y1 + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 2)

                if p.pos_map is not None:
                    Xm, Ym, Zm, yaw_deg = p.pos_map
                    cv2.putText(color_img, f"MAP: [{Xm:.2f}, {Ym:.2f}] yaw={yaw_deg:.0f}",
                                (x1, y1 + 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                cv2.putText(color_img, f"Size: {p.width_m*100:.1f}x{p.height_m*100:.1f}cm",
                            (x1, y1 + 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            gx, gy = p.grasp_point_px
            cv2.circle(color_img, (gx, gy), 5, (0, 0, 255), -1)

        # Barcodes
        for b in barcodes:
            cx, cy = b.center_px
            if b.corners and len(b.corners) >= 2:
                for i in range(len(b.corners)):
                    cv2.line(color_img, b.corners[i], b.corners[(i + 1) % len(b.corners)],
                             (255, 0, 255), 2)

            cv2.putText(color_img, f"{b.format_type}: {b.text}",
                        (max(0, cx - 120), max(0, cy - 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 220, 255), 2)

            if b.map_pose:
                mx, my, mtheta = b.map_pose
                cv2.putText(color_img, f"LANDMARK MAP: ({mx:.1f},{my:.1f},{mtheta:.0f}deg)",
                            (max(0, cx - 120), cy + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                cv2.putText(color_img, "UNREGISTERED",
                            (max(0, cx - 120), cy + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # IMU + FPS
        cv2.putText(color_img, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)

        if imu_euler is not None:
            r, p, y = imu_euler
            cv2.putText(color_img, f"IMU rpy(deg): {r:.0f} {p:.0f} {y:.0f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if self.base_pose_map is not None and imu_euler is not None:
            bx, by, th = self.base_pose_map
            yaw_used = imu_euler[2] + self.yaw_offset_deg
            cv2.putText(color_img, f"BASE MAP: ({bx:.2f},{by:.2f}) yaw_used={yaw_used:.0f}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return color_img

    def run_demo(self):
        print("[INFO] Starting demo. Press 'q' to quit.")
        print("[INFO] CAMERA: X=right, Y=down, Z=forward")
        print("[INFO] BASE  : X=forward, Y=left, Z=up (with cam_offset/cam_rpy)")
        if self.use_imu:
            print("[INFO] IMU enabled. Barcode landmark will align yaw -> map theta.")
        if self.udp_target:
            print(f"[INFO] UDP streaming enabled -> {self.udp_target[0]}:{self.udp_target[1]}")
        print("-" * 60)

        try:
            while True:
                color_img, products, barcodes, fps, imu_euler = self.process_frame()
                if color_img is None:
                    continue

                # Build + send payload to Jetson
                payload = self._build_payload(products, barcodes, fps, imu_euler)
                self._send_udp(payload)

                # Console prints (debug)
                for p in products:
                    print(f"[PRODUCT] {p}")

                for b in barcodes:
                    if b.map_pose:
                        mx, my, mtheta = b.map_pose
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                              f"Barcode '{b.text}' -> LANDMARK MAP "
                              f"x={mx:.2f}m, y={my:.2f}m, theta={mtheta:.1f}deg "
                              f"| range={b.depth_m:.2f}m")

                display_img = self.draw_results(color_img.copy(), products, barcodes, fps, imu_euler)
                cv2.imshow("Grocery Robot Vision", display_img)

                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break
        finally:
            self.stop()

    def stop(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if self.udp_sock:
            try:
                self.udp_sock.close()
            except Exception:
                pass
        print("[INFO] Vision system stopped.")


# ============ Main Function ============

def main():
    parser = argparse.ArgumentParser(description="Grocery Robot Vision System (Base/Map/IMU/Jetson)")
    parser.add_argument("--model", type=str, default="yolov8n.pt",
                        help="YOLO model path (default: yolov8n.pt)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Detection confidence threshold (default: 0.5)")

    # NEW: IMU + extrinsic + UDP
    parser.add_argument("--use_imu", action="store_true",
                        help="Enable D435i IMU (gyro+accel) and yaw alignment via barcode landmarks")
    parser.add_argument("--cam_offset", type=str, default="0,0,0",
                        help="Camera translation in BASE frame (meters): 'x,y,z' e.g. '0.12,0,0.35'")
    parser.add_argument("--cam_rpy", type=str, default="0,0,0",
                        help="Camera rotation relative to BASE (deg): 'roll,pitch,yaw' e.g. '0,-15,0'")
    parser.add_argument("--udp", type=str, default="",
                        help="Optional UDP target ip:port to stream JSON to Jetson, e.g. '192.168.1.88:5005'")

    args = parser.parse_args()

    cam_offset_base = parse_vec3_csv(args.cam_offset)
    cam_rpy_deg = parse_vec3_csv(args.cam_rpy)
    udp_target = parse_host_port(args.udp) if args.udp else None

    vision = GroceryRobotVision(
        yolo_model_path=args.model,
        conf_threshold=args.conf,
        use_imu=args.use_imu,
        cam_offset_base=cam_offset_base,
        cam_rpy_deg=cam_rpy_deg,
        udp_target=udp_target
    )
    vision.run_demo()


if __name__ == "__main__":
    main()
