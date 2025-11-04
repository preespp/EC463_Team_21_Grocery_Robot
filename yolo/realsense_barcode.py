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

def median_depth(depth_frame, x, y, k=7):
    """Take the median depth (in meters) of the kxk region around (x,y), ignoring zero values."""
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k//2), max(0, y - k//2)
    x1, y1 = min(w, x + k//2), min(h, y + k//2)
    vals = []
    for yy in range(y0, y1):
        for xx in range(x0, x1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0

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

def main():
    # 1) RealSense initialization (color 1280x720, depth 848x480)
    pipe = rs.pipeline()
    cfg  = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)

    fps_hist = deque(maxlen=20)

    # 2) Warm up a few frames
    for _ in range(5):
        pipe.wait_for_frames(10000)

    try:
        while True:
            t0 = time.time()
            frames  = pipe.wait_for_frames(10000)
            aligned = align.process(frames)
            depth   = aligned.get_depth_frame()
            color   = aligned.get_color_frame()
            if not depth or not color:
                continue

            color_img = np.asanyarray(color.get_data())

            # 3) ZXing decoding (for the entire frame)
            results = zxingcpp.read_barcodes(color_img)

            for r in results:
                pts = to_int_pts(r.position)  # Corner point
                # Draw a polygonal box
                if pts and len(pts) >= 2:
                    for i in range(len(pts)):
                        cv2.line(color_img, pts[i], pts[(i+1) % len(pts)], (0,255,0), 2)

                # Find the center point for distance measurement
                cx = int(np.mean([p[0] for p in pts])) if pts else color_img.shape[1]//2
                cy = int(np.mean([p[1] for p in pts])) if pts else color_img.shape[0]//2

                # Median depth (meters) and 3D
                dist_m = median_depth(depth, cx, cy, k=9)
                depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin,
                                                          [float(cx), float(cy)],
                                                          float(dist_m))

                # Text
                info_line1 = f"{r.format}: {r.text}"
                cv2.putText(color_img, info_line1, (max(0, cx-120), max(0, cy-14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50,220,255), 2)

                if dist_m > 0:
                    info_line2 = f"{dist_m:.2f} m | 3D:[{X:.2f},{Y:.2f},{Z:.2f}]"
                    cv2.putText(color_img, info_line2, (max(0, cx-120), cy+10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 2)

                # 4) Look up the table to get the map location (== "see the barcode and you know where you are")
                if r.text in BARCODE_MAP:
                    map_x, map_y, map_theta = BARCODE_MAP[r.text]

                    # -- Simple Strategy (Recommended for Starting) --
                    # Directly set "Current Location" to the map coordinates of the landmark (i.e., arrival location).
                    # If you need more refinement (e.g., determining front-to-back deviation based on Z), you can make fine adjustments here.
                    current_pose = (map_x, map_y, map_theta)

                    # Print
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                          f"Seen barcode '{r.text}' -> Robot at MAP "
                          f"x={current_pose[0]:.2f} m, y={current_pose[1]:.2f} m, "
                          f"theta={current_pose[2]:.1f} deg | range≈{dist_m:.2f} m")
                    
                else:
                    # Barcodes not registered on the map
                    cv2.putText(color_img, "UNREGISTERED BARCODE",
                                (max(0, cx-120), cy+30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)

            # FPS
            fps = 1.0 / max(1e-6, (time.time() - t0))
            fps_hist.append(fps)
            cv2.putText(color_img, f"FPS: {np.mean(fps_hist):.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,200,255), 2)

            cv2.imshow("ZXing + RealSense (Barcode -> Position)", color_img)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
