import argparse  # NEW
import time
import csv
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
import zxingcpp

# Barcode content -> Map coordinates (x, y, deg)
# Unit: Meters / Degrees
BARCODE_MAP = {
    "Location_1": (3.00, 4.00, 0.0),
    "Location_2": (1.50, 1.20, 0.0),
    "Location_3": (0.50, 2.40, 90.0),
}

def median_depth_np(depth_frame, x, y, k, depth_scale):  # NEW: NumPy版中值深度（米）
    arr = np.asanyarray(depth_frame.get_data())          # uint16, 单位=raw depth
    h, w = arr.shape
    x0, x1 = max(0, x - k//2), min(w, x + k//2 + 1)
    y0, y1 = max(0, y - k//2), min(h, y + k//2 + 1)
    win = arr[y0:y1, x0:x1]
    nz = win[win > 0]
    if nz.size == 0:
        return 0.0
    return float(np.median(nz) * depth_scale)            # 转换为米

def to_int_pts(position):
    """
    Extract corner points from a zxingcpp position object.
    For 2D codes (QR), it has top_left/top_right/bottom_right/bottom_left
    """
    pts = []
    for attr in ("top_left", "top_right", "bottom_right", "bottom_left"):
        if hasattr(position, attr):
            p = getattr(position, attr)
            if hasattr(p, "x") and hasattr(p, "y"):
                pts.append((int(p.x), int(p.y)))
    if pts:
        return pts

def scale_points(pts, sx, sy):  # NEW: 把在缩放图上的点映射回原图
    return [(int(px * sx), int(py * sy)) for (px, py) in pts]

def decode_frame_for_barcodes(color_img, max_decode_width=640):  # NEW: 灰度+下采样解码
    # 转灰度，ZXing对灰度就能解
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape

    # 如宽度超过限制，按比例缩小；记录比例用于回映射
    if w > max_decode_width:
        scale = max_decode_width / float(w)
        small = cv2.resize(gray, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        inv_sx = w / float(small.shape[1])  # 反比例
        inv_sy = h / float(small.shape[0])
        results_small = zxingcpp.read_barcodes(small)
        results = []
        for r in results_small:
            pts = to_int_pts(r.position)
            if pts:
                pts_full = scale_points(pts, inv_sx, inv_sy)
                # 构造一个“影子”对象：复用 r 的其他字段，替换角点
                class _R:
                    pass
                _r = _R()
                _r.text = r.text
                _r.format = r.format
                _r.position = r.position
                _r.pts_full = pts_full  # 自定义字段，后续使用
                results.append(_r)
        return results
    else:
        # 不缩放，直接返回
        results = zxingcpp.read_barcodes(gray)
        # 包装以统一字段名
        wrapped = []
        for r in results:
            class _R:
                pass
            _r = _R()
            _r.text = r.text
            _r.format = r.format
            _r.position = r.position
            _r.pts_full = to_int_pts(r.position)
            wrapped.append(_r)
        return wrapped

def main():
    parser = argparse.ArgumentParser()  # NEW
    parser.add_argument("--width", type=int, default=640, help="color/depth 宽度（统一尺寸）")
    parser.add_argument("--height", type=int, default=480, help="color/depth 高度（统一尺寸）")
    parser.add_argument("--k", type=int, default=7, help="中值深度窗口尺寸 k×k")
    parser.add_argument("--decode-interval", type=int, default=2,
                        help="每隔 N 帧解码一次（减负），N=1 表示每帧解码")
    parser.add_argument("--max-decode-width", type=int, default=640,
                        help="条码解码时的最大图像宽度（下采样加速）")
    parser.add_argument("--no-show", action="store_true", help="不显示窗口（进一步提速）")
    args = parser.parse_args()

    # 可选：限制 OpenCV 线程以避免在小核上抢占（按需开启）
    try:
        cv2.setNumThreads(1)  # NEW(可选)
    except Exception:
        pass

    # 1) RealSense initialization —— 统一分辨率，减少对齐成本（CHG）
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, 30)   # CHG
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, 30)  # CHG
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)

    # 深度尺度（把 uint16 转米用） —— NEW
    depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()

    fps_hist = deque(maxlen=20)

    # 2) Warm up a few frames
    for _ in range(5):
        pipe.wait_for_frames(10000)

    frame_idx = 0  # NEW
    last_results = []  # NEW: 可选保留上次结果用于 UI 持续显示

    try:
        while True:
            t0 = time.time()
            frames  = pipe.wait_for_frames(10000)
            aligned = align.process(frames)
            depth   = aligned.get_depth_frame()
            color   = aligned.get_color_frame()
            if not depth or not color:
                continue

            color_img = np.ascontiguousarray(np.asanyarray(color.get_data()))  # NEW

            # 3) ZXing decoding（间隔解码 + 下采样） —— NEW
            if frame_idx % max(1, args.decode_interval) == 0:
                results = decode_frame_for_barcodes(color_img, max_decode_width=args.max_decode_width)
                last_results = results
            else:
                results = last_results

            # 遍历检测结果
            for r in results:
                pts = getattr(r, "pts_full", None)
                # 画多边形框
                if pts and len(pts) >= 2:
                    for i in range(len(pts)):
                        cv2.line(color_img, pts[i], pts[(i+1) % len(pts)], (0,255,0), 2)

                # 用角点中心估计测距点
                if pts and len(pts) >= 1:
                    cx = int(np.mean([p[0] for p in pts]))
                    cy = int(np.mean([p[1] for p in pts]))
                else:
                    h, w, _ = color_img.shape
                    cx, cy = w // 2, h // 2

                # 中值深度（米）+ 3D 反投影 —— 使用 NumPy 版，减少循环开销
                dist_m = median_depth_np(depth, cx, cy, k=args.k, depth_scale=depth_scale)
                depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()
                X, Y, Z = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, [float(cx), float(cy)], float(dist_m)
                )

                # 文本
                info_line1 = f"{getattr(r, 'format', 'UNK')}: {getattr(r, 'text', '')}"
                cv2.putText(color_img, info_line1, (max(0, cx-120), max(0, cy-14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,220,255), 2)

                if dist_m > 0:
                    info_line2 = f"{dist_m:.2f} m | 3D:[{X:.2f},{Y:.2f},{Z:.2f}]"
                    cv2.putText(color_img, info_line2, (max(0, cx-120), cy+10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)

                # 4) 查表得到地图位置
                txt = getattr(r, "text", "")
                if txt in BARCODE_MAP:
                    map_x, map_y, map_theta = BARCODE_MAP[txt]
                    current_pose = (map_x, map_y, map_theta)
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Seen barcode '{txt}' -> Robot at MAP "
                          f"x={current_pose[0]:.2f} m, y={current_pose[1]:.2f} m, "
                          f"theta={current_pose[2]:.1f} deg | range≈{dist_m:.2f} m")
                else:
                    cv2.putText(color_img, "UNREGISTERED BARCODE",
                                (max(0, cx-120), cy+30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

            # FPS
            fps = 1.0 / max(1e-6, (time.time() - t0))
            fps_hist.append(fps)
            cv2.putText(color_img, f"FPS: {np.mean(fps_hist):.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,200,255), 2)

            if not args.no_show:  # NEW
                cv2.imshow("ZXing + RealSense (Barcode -> Position)", color_img)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break

            frame_idx += 1  # NEW
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
