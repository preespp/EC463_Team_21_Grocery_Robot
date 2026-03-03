import argparse
import json
import math
import time

import cv2
import numpy as np
import pyrealsense2 as rs
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

        self.roll += gx * dt
        self.pitch += gy * dt
        self.yaw += gz * dt

        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_acc
        self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_acc


def main():
    parser = argparse.ArgumentParser(
        description="D435i end-effector feedback demo (IMU pose + object 3D TF-like output)"
    )
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--conf", type=float, default=0.5)
    args = parser.parse_args()

    model = YOLO(args.model)
    imu_fusion = SimpleIMUFusion(alpha=0.98)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)

    last_accel = None
    last_gyro = None
    print("Running. Press q to quit.")

    try:
        while True:
            frames = pipe.wait_for_frames(timeout_ms=3000)
            for f in frames:
                st = f.get_profile().stream_type()
                if st == rs.stream.accel:
                    m = f.as_motion_frame().get_motion_data()
                    last_accel = (float(m.x), float(m.y), float(m.z))
                elif st == rs.stream.gyro:
                    m = f.as_motion_frame().get_motion_data()
                    last_gyro = (float(m.x), float(m.y), float(m.z))

            if last_accel is not None and last_gyro is not None:
                imu_fusion.update(last_accel, last_gyro)

            aligned = align.process(frames)
            depth = aligned.get_depth_frame()
            color = aligned.get_color_frame()
            if not depth or not color:
                continue

            img = np.asanyarray(color.get_data())
            depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()

            detections = []
            results = model.predict(source=img, conf=args.conf, iou=0.45, verbose=False)
            for r in results:
                names = r.names
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf = float(box.conf[0].cpu().numpy())
                    name = names.get(cls_id, str(cls_id))

                    cx = int((x1 + x2) * 0.5)
                    cy = int((y1 + y2) * 0.5)
                    dist_m = median_depth(depth, cx, cy, k=5)
                    if dist_m > 0.0:
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(
                            depth_intrin, [float(cx), float(cy)], float(dist_m)
                        )
                    else:
                        X = Y = Z = 0.0

                    detections.append(
                        {
                            "name": name,
                            "conf": conf,
                            "distance_m": float(dist_m),
                            "object_tf_camera_m": [float(X), float(Y), float(Z)],
                        }
                    )

                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        img,
                        f"{name} {dist_m:.2f}m",
                        (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

            payload = {
                "timestamp": time.time(),
                "camera_imu_feedback_rpy_rad": [
                    float(imu_fusion.roll),
                    float(imu_fusion.pitch),
                    float(imu_fusion.yaw),
                ],
                "detections": detections,
            }
            print(json.dumps(payload, ensure_ascii=False))

            cv2.imshow("realsense_feedback_demo", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
