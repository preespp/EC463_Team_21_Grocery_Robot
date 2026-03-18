import json
import math
import time
from collections import deque

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from ultralytics import YOLO


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


def robust_bbox_depth(depth_frame, x1, y1, x2, y2):
    """
    Get a robust depth estimate from a bbox.
    1) Try median depth at bbox center.
    2) If invalid, scan inside bbox for nearest valid depth.
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

    step = 2
    best_depth = float("inf")
    best_x = cx
    best_y = cy
    for yy in range(by1, by2 + 1, step):
        for xx in range(bx1, bx2 + 1, step):
            d = depth_frame.get_distance(xx, yy)
            if 0.08 < d < 3.0 and d < best_depth:
                best_depth = d
                best_x = xx
                best_y = yy

    if best_depth != float("inf"):
        return float(best_depth), int(best_x), int(best_y)

    return 0.0, cx, cy


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

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("use_cuda", True)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("use_imu", False)
        self.declare_parameter("depth_width", 848)
        self.declare_parameter("depth_height", 480)
        self.declare_parameter("depth_fps", 30)
        self.declare_parameter("color_width", 1280)
        self.declare_parameter("color_height", 720)
        self.declare_parameter("color_fps", 30)
        self.declare_parameter("parent_frame", "ee_link")
        self.declare_parameter("camera_feedback_frame", "camera_feedback")
        self.declare_parameter("object_frame_prefix", "object")
        self.declare_parameter("mount_xyz", "-0.0635,0.0,0.0635")
        self.declare_parameter("mount_rpy_deg", "0.0,0.0,0.0")
        self.declare_parameter("max_objects_tf", 5)

        model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf").value)
        use_cuda = bool(self.get_parameter("use_cuda").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)
        self.use_imu = bool(self.get_parameter("use_imu").value)
        self.depth_width = int(self.get_parameter("depth_width").value)
        self.depth_height = int(self.get_parameter("depth_height").value)
        self.depth_fps = int(self.get_parameter("depth_fps").value)
        self.color_width = int(self.get_parameter("color_width").value)
        self.color_height = int(self.get_parameter("color_height").value)
        self.color_fps = int(self.get_parameter("color_fps").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.camera_feedback_frame = str(self.get_parameter("camera_feedback_frame").value)
        self.object_frame_prefix = str(self.get_parameter("object_frame_prefix").value)
        self.max_objects_tf = int(self.get_parameter("max_objects_tf").value)

        self.mount_xyz = self._parse_vec3(str(self.get_parameter("mount_xyz").value))
        roll_deg, pitch_deg, yaw_deg = self._parse_vec3(
            str(self.get_parameter("mount_rpy_deg").value)
        )
        self.mount_quat = euler_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        )

        self.pub_detections = self.create_publisher(String, "detections_json", 10)
        self.pub_point = self.create_publisher(PointStamped, "detection_point", 10)
        if self.publish_image:
            self.pub_image = self.create_publisher(Image, "camera/color/image_raw", 10)
            self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
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
                "Primary RealSense profile failed (%s). Falling back to 640x480@30 without IMU.",
                str(exc),
            )
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(cfg)
            self.use_imu = False
        self.align_to_color = rs.align(rs.stream.color)

        self.imu_fusion = SimpleIMUFusion(alpha=0.98)
        self.latest_accel = None
        self.latest_gyro = None
        self.fps_hist = deque(maxlen=20)
        self.last_t = time.time()

        self.timer = self.create_timer(0.02, self.timer_cb)

    @staticmethod
    def _parse_vec3(text):
        parts = [p.strip() for p in text.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Expected 3 comma-separated values, got: {text}")
        return float(parts[0]), float(parts[1]), float(parts[2])

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

    def _broadcast_camera_feedback_tf(self, stamp):
        imu_quat = euler_to_quat(
            self.imu_fusion.roll, self.imu_fusion.pitch, self.imu_fusion.yaw
        )
        q = quat_multiply(self.mount_quat, imu_quat)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.camera_feedback_frame
        t.transform.translation.x = float(self.mount_xyz[0])
        t.transform.translation.y = float(self.mount_xyz[1])
        t.transform.translation.z = float(self.mount_xyz[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_object_tf(self, stamp, detections):
        for i, det in enumerate(detections[: self.max_objects_tf]):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.camera_feedback_frame
            t.child_frame_id = f"{self.object_frame_prefix}_{i}"
            t.transform.translation.x = float(det["point_m"][0])
            t.transform.translation.y = float(det["point_m"][1])
            t.transform.translation.z = float(det["point_m"][2])
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

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
                name = names.get(cls_id, str(cls_id))

                dist_m, cx, cy = robust_bbox_depth(depth, x1, y1, x2, y2)
                if dist_m > 0.0:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(
                        depth_intrin, [float(cx), float(cy)], float(dist_m)
                    )
                else:
                    X = Y = Z = 0.0

                det = {
                    "class_id": cls_id,
                    "class_name": name,
                    "confidence": conf,
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center_px": [cx, cy],
                    "distance_m": float(dist_m),
                    "point_m": [float(X), float(Y), float(Z)],
                    "grasp_px": [cx, cy],
                }
                detections_out.append(det)

                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = self.camera_feedback_frame
                pt_msg.point.x = float(X)
                pt_msg.point.y = float(Y)
                pt_msg.point.z = float(Z)
                self.pub_point.publish(pt_msg)

                if self.publish_image:
                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{name} {conf:.2f} {dist_m:.2f}m"
                    cv2.putText(
                        color_img,
                        label,
                        (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

        stamp = self.get_clock().now().to_msg()
        self._broadcast_camera_feedback_tf(stamp)
        if detections_out:
            self._broadcast_object_tf(stamp, detections_out)

        payload = {
            "timestamp": time.time(),
            "camera_feedback_frame": self.camera_feedback_frame,
            "imu_rpy_rad": [
                float(self.imu_fusion.roll),
                float(self.imu_fusion.pitch),
                float(self.imu_fusion.yaw),
            ],
            "detections": detections_out,
        }
        self.pub_detections.publish(String(data=json.dumps(payload)))

        if self.publish_image:
            ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
            ros_img.header.stamp = stamp
            ros_img.header.frame_id = self.camera_feedback_frame
            self.pub_image.publish(ros_img)

        now = time.time()
        fps = 1.0 / max(1e-6, now - self.last_t)
        self.last_t = now
        self.fps_hist.append(fps)
        if len(self.fps_hist) == self.fps_hist.maxlen:
            self.get_logger().debug(
                f"FPS={sum(self.fps_hist)/len(self.fps_hist):.1f}, "
                f"detections={len(detections_out)}"
            )

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
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
