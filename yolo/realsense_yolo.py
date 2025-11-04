import argparse
import time
from collections import deque

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

"""
Windows + RealSense D435 + YOLO 实时检测并回报每个目标的距离 & 3D 坐标。
用法:
  python realsense_yolo_infer.py --model yolov8n.pt --conf 0.5
说明:
  - 默认彩色 1280x720@30，深度 848x480@30，并把深度对齐到彩色坐标系。
  - 距离采用 bbox 中心 5x5 小窗口的“中值深度”，更稳健。
  - 3D 点为相机坐标系 (X,Y,Z) 米；X向右、Y向下、Z向前（离相机的距离≈Z）。
"""

def median_depth(depth_frame, x, y, k=5):
    """在 (x,y) 周围取 kxk 区域的中值深度（米），忽略0值。"""
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k//2), max(0, y - k//2)
    x1, y1 = min(w, x + k//2), min(h, y + k//2)
    vals = []
    for yy in range(y0, y1):
        for xx in range(x0, x1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    if not vals:
        return 0.0
    return float(np.median(vals))

def main(args):
    # 1) 初始化 RealSense
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipe.start(cfg)

    # 将深度帧对齐到彩色帧坐标系
    align_to_color = rs.align(rs.stream.color)

    # 2) 加载 YOLO（可换成你训练好的 .pt）
    model = YOLO(args.model)

    fps_hist = deque(maxlen=20)
    try:
        while True:
            t0 = time.time()
            frames  = pipe.wait_for_frames()
            aligned = align_to_color.process(frames)
            depth   = aligned.get_depth_frame()
            color   = aligned.get_color_frame()
            if not depth or not color:
                continue

            color_img = np.asanyarray(color.get_data())

            # 3) YOLO 推理（BGR 图像直接喂）
            results = model(color_img, conf=args.conf, iou=0.45, verbose=False)

            # 4) 遍历检测框，计算距离 & 3D 坐标
            for r in results:
                names = r.names  # 类别名字典
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf   = float(box.conf[0].cpu().numpy())
                    name   = names.get(cls_id, str(cls_id))

                    # 取框中心像素
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # 中值深度（米）
                    dist_m = median_depth(depth, cx, cy, k=5)

                    # 反投影到3D：使用对齐后的“深度帧内参”
                    depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin,
                                                              [float(cx), float(cy)],
                                                              dist_m)

                    # 画框与文字
                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{name} {conf:.2f}"
                    if dist_m > 0:
                        label += f" | {dist_m:.2f}m"
                    cv2.putText(color_img, label, (x1, max(0, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    if dist_m > 0:
                        cv2.putText(color_img, f"3D: [{X:.2f}, {Y:.2f}, {Z:.2f}] m",
                                    (x1, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)

            # 5) FPS
            fps = 1.0 / max(1e-6, (time.time() - t0))
            fps_hist.append(fps)
            cv2.putText(color_img, f"FPS: {np.mean(fps_hist):.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)

            cv2.imshow("YOLO + RealSense D435 (Aligned Depth to Color)", color_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=str, default="yolov8n.pt",
                    help="YOLO 权重（.pt），默认 COCO 预训练 yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    args = ap.parse_args()
    main(args)