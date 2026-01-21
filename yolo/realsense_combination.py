"""
Grocery Robot Vision System
============================
Features:
  1. YOLO product detection + 3D position + size estimation (for robotic arm grasping)
  2. ZXing barcode scanning + robot localization

Usage:
  python realsense_grocery_robot.py --model yolov8n.pt --conf 0.5

Output for each detected product:
  - name: product class name
  - confidence: detection confidence (0-1)
  - bbox: 2D bounding box (x1, y1, x2, y2) in pixels
  - center_px: center point (cx, cy) in pixels
  - depth_m: distance from camera in meters
  - pos_3d: 3D position (X, Y, Z) in camera frame (meters)
           X: right, Y: down, Z: forward
  - width_m: estimated object width in meters
  - height_m: estimated object height in meters
"""

import argparse
import time
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
    
    # Sample points and find minimum valid depth
    min_depth = float('inf')
    best_x, best_y = (bx1 + bx2) // 2, (by1 + by2) // 2
    
    for yy in range(by1, by2, 2):  # Step by 2 for speed
        for xx in range(bx1, bx2, 2):
            d = depth_frame.get_distance(xx, yy)
            if 0.1 < d < 3.0 and d < min_depth:  # Valid range: 10cm - 3m
                min_depth = d
                best_x, best_y = xx, yy
    
    if min_depth == float('inf'):
        # Fallback to center point
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        return median_depth(depth_frame, cx, cy, k=5), cx, cy
    
    return min_depth, best_x, best_y


def get_barcode_corners(position):
    """
    Extract four corner coordinates from a zxingcpp position object.
    Returns list of [(x, y), ...].
    """
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
    Returns (X, Y, Z) in meters.
    """
    X, Y, Z = rs.rs2_deproject_pixel_to_point(
        depth_intrin, [float(cx), float(cy)], float(dist_m)
    )
    return X, Y, Z


# ============ Detection Result Classes ============

class DetectedProduct:
    """
    Detected product/item
    
    Attributes:
        name: Product class name (e.g., "bottle", "apple")
        confidence: Detection confidence 0-1
        bbox: 2D bounding box (x1, y1, x2, y2) in pixels
        center_px: Center point (cx, cy) in pixels
        depth_m: Distance from camera in meters
        pos_3d: 3D position (X, Y, Z) in camera frame
                X: right positive, Y: down positive, Z: forward positive
        width_m: Estimated object width in meters
        height_m: Estimated object height in meters
        grasp_point_px: Best grasp point (x, y) in pixels (closest point)
    """
    def __init__(self, name, confidence, bbox, center_px, depth_m, pos_3d,
                 width_m=0.0, height_m=0.0, grasp_point_px=None):
        self.name = name
        self.confidence = confidence
        self.bbox = bbox
        self.center_px = center_px
        self.depth_m = depth_m
        self.pos_3d = pos_3d
        self.width_m = width_m
        self.height_m = height_m
        self.grasp_point_px = grasp_point_px or center_px
    
    def __repr__(self):
        return (f"DetectedProduct(name='{self.name}', conf={self.confidence:.2f}, "
                f"pos=[{self.pos_3d[0]:.3f}, {self.pos_3d[1]:.3f}, {self.pos_3d[2]:.3f}]m, "
                f"size=[{self.width_m*100:.1f}x{self.height_m*100:.1f}]cm)")


class DetectedBarcode:
    """
    Detected barcode
    
    Attributes:
        text: Barcode content string
        format_type: Barcode format (e.g., "QRCode", "Code128")
        corners: Four corner points [(x,y), ...]
        center_px: Center point in pixels
        depth_m: Distance from camera in meters
        pos_3d: 3D position in camera frame
        map_pose: Map coordinates (x, y, theta) or None if unregistered
    """
    def __init__(self, text, format_type, corners, center_px, depth_m, pos_3d, map_pose=None):
        self.text = text
        self.format_type = format_type
        self.corners = corners
        self.center_px = center_px
        self.depth_m = depth_m
        self.pos_3d = pos_3d
        self.map_pose = map_pose
    
    def __repr__(self):
        if self.map_pose:
            return f"DetectedBarcode(text='{self.text}', map_pose={self.map_pose})"
        return f"DetectedBarcode(text='{self.text}', UNREGISTERED)"


# ============ Main Vision Class ============

class GroceryRobotVision:
    """
    Grocery Robot Vision System
    Integrates YOLO product detection + ZXing barcode localization
    
    Output coordinate system (camera frame):
        X: right is positive
        Y: down is positive  
        Z: forward is positive (distance from camera)
    """

    def __init__(self, yolo_model_path="yolov8n.pt", conf_threshold=0.5):
        self.conf_threshold = conf_threshold

        # Initialize RealSense
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)

        # Initialize YOLO
        print(f"[INFO] Loading YOLO model: {yolo_model_path}")
        self.yolo_model = YOLO(yolo_model_path)

        # FPS statistics
        self.fps_hist = deque(maxlen=20)

        # Warm up camera
        print("[INFO] Warming up camera...")
        for _ in range(10):
            self.pipe.wait_for_frames(10000)
        print("[INFO] Vision system ready!")

    def get_frames(self):
        """Get aligned depth and color frames"""
        frames = self.pipe.wait_for_frames(10000)
        aligned = self.align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        return depth, color

    def detect_products(self, color_img, depth_frame):
        """
        Detect products using YOLO
        Returns list of DetectedProduct with 3D position and size estimation
        """
        products = []
        
        # Get camera intrinsics for 3D projection and size estimation
        depth_intrin = depth_frame.profile.as_video_stream_profile().get_intrinsics()
        fx = depth_intrin.fx  # Focal length x
        fy = depth_intrin.fy  # Focal length y
        
        results = self.yolo_model(
            color_img, conf=self.conf_threshold, iou=0.45, verbose=False
        )

        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                name = names.get(cls_id, str(cls_id))

                # Get depth using closest point method (better for grasping)
                depth_m, grasp_x, grasp_y = get_grasp_depth(depth_frame, x1, y1, x2, y2)
                
                # Center point (for display)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                
                # 3D position of grasp point
                if depth_m > 0:
                    pos_3d = pixel_to_3d(depth_intrin, grasp_x, grasp_y, depth_m)
                else:
                    pos_3d = (0, 0, 0)

                # Estimate real-world size from bbox and depth
                width_px = x2 - x1
                height_px = y2 - y1
                
                if depth_m > 0 and fx > 0 and fy > 0:
                    width_m = width_px * depth_m / fx
                    height_m = height_px * depth_m / fy
                else:
                    width_m, height_m = 0.0, 0.0

                products.append(DetectedProduct(
                    name=name,
                    confidence=conf,
                    bbox=(x1, y1, x2, y2),
                    center_px=(cx, cy),
                    depth_m=depth_m,
                    pos_3d=pos_3d,
                    width_m=width_m,
                    height_m=height_m,
                    grasp_point_px=(grasp_x, grasp_y)
                ))

        return products

    def detect_barcodes(self, color_img, depth_frame):
        """
        Detect barcodes using ZXing
        Returns list of DetectedBarcode
        """
        barcodes = []
        
        # Get camera intrinsics
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
            pos_3d = pixel_to_3d(depth_intrin, cx, cy, dist_m) if dist_m > 0 else (0, 0, 0)

            # Look up map coordinates
            map_pose = BARCODE_MAP.get(r.text, None)

            barcodes.append(DetectedBarcode(
                text=r.text,
                format_type=str(r.format),
                corners=corners,
                center_px=(cx, cy),
                depth_m=dist_m,
                pos_3d=pos_3d,
                map_pose=map_pose
            ))

        return barcodes

    def process_frame(self):
        """
        Process one frame: detect products and barcodes
        Returns (color_img, products, barcodes, fps)
        """
        t0 = time.time()

        depth, color = self.get_frames()
        if not depth or not color:
            return None, [], [], 0

        color_img = np.asanyarray(color.get_data())

        # Run detections
        products = self.detect_products(color_img, depth)
        barcodes = self.detect_barcodes(color_img, depth)

        # Calculate FPS
        fps = 1.0 / max(1e-6, time.time() - t0)
        self.fps_hist.append(fps)

        return color_img, products, barcodes, np.mean(self.fps_hist)

    def draw_results(self, color_img, products, barcodes, fps):
        """Draw detection results on image"""
        # Draw products
        for p in products:
            x1, y1, x2, y2 = p.bbox
            cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Label with name, confidence, depth
            label = f"{p.name} {p.confidence:.2f}"
            if p.depth_m > 0:
                label += f" | {p.depth_m:.2f}m"
            cv2.putText(color_img, label, (x1, max(0, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 3D position and size
            if p.depth_m > 0:
                X, Y, Z = p.pos_3d
                cv2.putText(color_img, f"3D: [{X:.2f}, {Y:.2f}, {Z:.2f}]",
                            (x1, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(color_img, f"Size: {p.width_m*100:.1f}x{p.height_m*100:.1f}cm",
                            (x1, y1 + 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Draw grasp point
            gx, gy = p.grasp_point_px
            cv2.circle(color_img, (gx, gy), 5, (0, 0, 255), -1)  # Red dot

        # Draw barcodes
        for b in barcodes:
            cx, cy = b.center_px

            # Draw polygon outline
            if b.corners and len(b.corners) >= 2:
                for i in range(len(b.corners)):
                    cv2.line(color_img, b.corners[i], b.corners[(i + 1) % len(b.corners)],
                             (255, 0, 255), 2)

            # Barcode info
            cv2.putText(color_img, f"{b.format_type}: {b.text}",
                        (max(0, cx - 120), max(0, cy - 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 220, 255), 2)

            if b.depth_m > 0:
                X, Y, Z = b.pos_3d
                cv2.putText(color_img, f"{b.depth_m:.2f}m | [{X:.2f},{Y:.2f},{Z:.2f}]",
                            (max(0, cx - 120), cy + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Map position
            if b.map_pose:
                mx, my, mtheta = b.map_pose
                cv2.putText(color_img, f"MAP: ({mx:.1f}, {my:.1f}, {mtheta:.0f}deg)",
                            (max(0, cx - 120), cy + 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            else:
                cv2.putText(color_img, "UNREGISTERED",
                            (max(0, cx - 120), cy + 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # FPS display
        cv2.putText(color_img, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)

        return color_img

    def run_demo(self):
        """Run real-time demo window"""
        print("[INFO] Starting demo. Press 'q' to quit.")
        print("[INFO] Coordinate system: X=right, Y=down, Z=forward (camera frame)")
        print("-" * 60)
        
        try:
            while True:
                color_img, products, barcodes, fps = self.process_frame()
                if color_img is None:
                    continue

                # Print detected products (for arm)
                for p in products:
                    print(f"[PRODUCT] {p}")

                # Print detected barcodes (for localization)
                for b in barcodes:
                    if b.map_pose:
                        mx, my, mtheta = b.map_pose
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] "
                              f"Barcode '{b.text}' -> Robot at MAP "
                              f"x={mx:.2f}m, y={my:.2f}m, theta={mtheta:.1f}deg "
                              f"| range={b.depth_m:.2f}m")

                # Draw and display
                display_img = self.draw_results(color_img.copy(), products, barcodes, fps)
                cv2.imshow("Grocery Robot Vision", display_img)

                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break
        finally:
            self.stop()

    def stop(self):
        """Stop camera and cleanup"""
        self.pipe.stop()
        cv2.destroyAllWindows()
        print("[INFO] Vision system stopped.")


# ============ Main Function ============

def main():
    parser = argparse.ArgumentParser(description="Grocery Robot Vision System")
    parser.add_argument("--model", type=str, default="yolov8n.pt",
                        help="YOLO model path (default: yolov8n.pt)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Detection confidence threshold (default: 0.5)")
    args = parser.parse_args()

    vision = GroceryRobotVision(
        yolo_model_path=args.model,
        conf_threshold=args.conf
    )
    vision.run_demo()


if __name__ == "__main__":
    main()