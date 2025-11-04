import argparse
import time
from collections import deque

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import torch  # NEW

"""
Windows + RealSense D435 + YOLO 实时检测并回报每个目标的距离 & 3D 坐标。
用法:
  python realsense_yolo_infer.py --model yolov8n.pt --conf 0.5
说明:
  - 默认彩色 1280x720@30，深度 848x480@30，并把深度对齐到彩色坐标系。
  - 距离采用 bbox 中心 5x5 小窗口的“中值深度”，更稳健。
  - 3D 点为相机坐标系 (X,Y,Z) 米；X向右、Y向下、Z向前（离相机的距离≈Z）。
"""

def median_depth_np(depth_frame, x, y, k, depth_scale):  # NEW: 用 numpy 求中值
    arr = np.asanyarray(depth_frame.get_data())          # uint16 深度单位
    h, w = arr.shape
    x0, x1 = max(0, x - k//2), min(w, x + k//2 + 1)
    y0, y1 = max(0, y - k//2), min(h, y + k//2 + 1)
    win = arr[y0:y1, x0:x1]
    nz = win[win > 0]
    if nz.size == 0:
        return 0.0
    return float(np.median(nz) * depth_scale)            # 转米

def main(args):
    # 0) 设备/半精度设置
    use_cuda = torch.cuda.is_available()                 # NEW
    device = 0 if use_cuda else 'cpu'                    # NEW
    print(f"[INFO] device={device}, cuda={use_cuda}")    # NEW

    # 1) 初始化 RealSense（降分辨率以减负）
    pipe = rs.pipeline()
    cfg  = rs.config()
    # 建议两路 640x480@30，减少对齐和拷贝开销                      # NEW(修改分辨率)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)

    # 深度尺度（把 uint16 转米用）
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()  # NEW

    # 将深度帧对齐到彩色帧坐标系
    align_to_color = rs.align(rs.stream.color)

    # 2) 加载 YOLO，指定 device / imgsz / half
    model = YOLO(args.model)
    if use_cuda:
        model.to('cuda')                                 # NEW
    # 预热（避免第一帧很慢）
    dummy = np.zeros((args.imgsz, args.imgsz, 3), dtype=np.uint8)  # NEW
    model.predict(dummy, device=device, imgsz=args.imgsz, half=use_cuda, verbose=False)  # NEW

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

            color_img = np.ascontiguousarray(np.asanyarray(color.get_data()))  # NEW

            # 3) YOLO 推理（指定 device/imgsz/half; 关闭梯度）
            with torch.inference_mode():               # NEW
                results = model.predict(
                    color_img,
                    conf=args.conf,
                    iou=0.45,
                    device=device,                    # NEW
                    imgsz=args.imgsz,                 # NEW (默认 416/512 比 640 更快)
                    half=use_cuda,                    # NEW
                    verbose=False
                )

            # 4) 遍历检测框，计算距离 & 3D 坐标
            for r in results:
                names = r.names
                for box in r.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cls_id = int(box.cls[0].cpu().numpy())
                    conf   = float(box.conf[0].cpu().numpy())
                    name   = names.get(cls_id, str(cls_id))

                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # 用 numpy 版中值深度
                    dist_m = median_depth_np(depth, cx, cy, k=5, depth_scale=depth_scale)  # NEW

                    # 反投影到3D
                    depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin,
                                                              [float(cx), float(cy)],
                                                              dist_m)

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

            if not args.no_show:  # NEW
                cv2.imshow("YOLO + RealSense D435 (Aligned Depth to Color)", color_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=str, default="yolov8n.pt",
                    help="YOLO 权重（.pt/.engine），默认 COCO 预训练 yolov8n.pt")
    ap.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    ap.add_argument("--imgsz", type=int, default=416, help="推理分辨率（如 320/416/512/640）")  # NEW
    ap.add_argument("--no-show", action="store_true", help="不显示窗口（提速）")                # NEW
    args = ap.parse_args()
    main(args)
