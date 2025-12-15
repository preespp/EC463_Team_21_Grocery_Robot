import time
import json
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Quaternion
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import math

# ZXing C++ Python bindings (zxingcpp)
import zxingcpp

# Map of barcode text -> (x, y, theta_deg) in MAP frame
BARCODE_MAP = {
    "Location_1": (3.00, 4.00, 0.0),
    "Location_2": (1.50, 1.20, 0.0),
    "Location_3": (0.50, 2.40, 90.0),
}

def median_depth(depth_frame, x, y, k=7):
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k//2), max(0, y - k//2)
    x1, y1 = min(w-1, x + k//2), min(h-1, y + k//2)
    vals = []
    for yy in range(y0, y1+1):
        for xx in range(x0, x1+1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    return float(np.median(vals)) if vals else 0.0

def euler_to_quaternion(yaw_deg):
    # yaw around Z; return geometry_msgs/Quaternion
    yaw = math.radians(yaw_deg)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # no roll/pitch
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)

def to_int_pts(position):
    pts = []
    for attr in ("top_left", "top_right", "bottom_right", "bottom_left"):
        if hasattr(position, attr):
            p = getattr(position, attr)
            if hasattr(p, "x") and hasattr(p, "y"):
                pts.append((int(p.x), int(p.y)))
    return pts

class RealSenseBarcodeNode(Node):
    def __init__(self, publish_image=True):
        super().__init__('realsense_barcode_node')
        self.get_logger().info("Starting RealSense Barcode Node")

        # publishers
        self.pub_json = self.create_publisher(String, 'barcode_detections_json', 10)
        self.pub_point = self.create_publisher(PointStamped, 'barcode_point', 10)
        self.pub_pose = self.create_publisher(PoseStamped, 'barcode_pose', 10)
        self.publish_image = publish_image
        if publish_image:
            self.pub_image = self.create_publisher(Image, 'camera/color/image_raw', 5)
            self.bridge = CvBridge()

        # RealSense setup
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(cfg)
        self.align = rs.align(rs.stream.color)

        self.fps_hist = deque(maxlen=20)
        # tiny periodic timer to keep node responsive
        self.timer = self.create_timer(0.001, self.timer_cb)
        self._last = time.time()

    def timer_cb(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        except Exception as e:
            self.get_logger().error(f"RealSense capture error: {e}")
            return

        aligned = self.align.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not (depth and color):
            return

        img = np.asanyarray(color.get_data())

        # ZXing decode: returns list of Barcode objects
        try:
            results = zxingcpp.read_barcodes(img)
        except Exception as e:
            self.get_logger().error(f"ZXing decode error: {e}")
            results = []

        detections = []
        depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()

        for res in results:
            # res.text, res.format, res.position
            pts = to_int_pts(res.position) or []
            if pts:
                cx = int(np.mean([p[0] for p in pts]))
                cy = int(np.mean([p[1] for p in pts]))
            else:
                cx, cy = img.shape[1]//2, img.shape[0]//2

            dist_m = median_depth(depth, cx, cy, k=9)

            if dist_m > 0:
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [float(cx), float(cy)], dist_m)
            else:
                X = Y = Z = 0.0

            det = {
                'text': res.text,
                'format': res.format,
                'center_px': [int(cx), int(cy)],
                'distance_m': float(dist_m),
                'point_camera_m': [float(X), float(Y), float(Z)],
                'registered': bool(res.text in BARCODE_MAP)
            }

            # If registered, add map pose
            if res.text in BARCODE_MAP:
                mx, my, mtheta = BARCODE_MAP[res.text]
                det['map_pose'] = {'x': float(mx), 'y': float(my), 'theta_deg': float(mtheta)}

                # publish PoseStamped (map frame)
                ps = PoseStamped()
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.header.frame_id = 'map'
                ps.pose = Pose()
                ps.pose.position.x = float(mx)
                ps.pose.position.y = float(my)
                ps.pose.position.z = 0.0
                ps.pose.orientation = euler_to_quaternion(mtheta)
                self.pub_pose.publish(ps)

            # publish point in camera frame
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = 'camera_link'
            pt.point.x = float(X)
            pt.point.y = float(Y)
            pt.point.z = float(Z)
            self.pub_point.publish(pt)

            detections.append(det)

            # Draw debug on image
            color_draw = (0, 255, 0) if det['registered'] else (0, 128, 255)
            if pts:
                for i in range(len(pts)):
                    cv2.line(img, pts[i], pts[(i+1) % len(pts)], color_draw, 2)
            cv2.putText(img, f"{res.text}", (max(0, cx-80), max(0, cy-10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_draw, 2)
            if dist_m > 0:
                cv2.putText(img, f"{dist_m:.2f}m", (cx-40, cy+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # Publish JSON if any
        if detections:
            msg = {'timestamp': time.time(), 'detections': detections}
            self.pub_json.publish(String(data=json.dumps(msg)))

        # Publish image for debugging/visualization
        if self.publish_image:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                ros_img.header.stamp = self.get_clock().now().to_msg()
                ros_img.header.frame_id = 'camera_link'
                self.pub_image.publish(ros_img)
            except Exception as e:
                self.get_logger().warning(f"cv_bridge error: {e}")

        # FPS logging
        now = time.time()
        fps = 1.0 / max(1e-6, (now - self._last))
        self._last = now
        self.fps_hist.append(fps)
        if len(self.fps_hist) == self.fps_hist.maxlen:
            self.get_logger().debug(f"Barcode FPS: {sum(self.fps_hist)/len(self.fps_hist):.1f}")

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseBarcodeNode(publish_image=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
