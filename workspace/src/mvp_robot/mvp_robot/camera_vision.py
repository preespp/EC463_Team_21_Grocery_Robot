import time
import json
from collections import deque
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import torch
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

def median_depth(depth_frame, x, y, k=5):
    """Get median depth (meters) of kxk window; ignore zeros."""
    h, w = depth_frame.get_height(), depth_frame.get_width()
    x0, y0 = max(0, x - k // 2), max(0, y - k // 2)
    x1, y1 = min(w - 1, x + k // 2), min(h - 1, y + k // 2)
    vals = []
    for yy in range(y0, y1 + 1):
        for xx in range(x0, x1 + 1):
            d = depth_frame.get_distance(xx, yy)
            if d > 0:
                vals.append(d)
    if not vals:
        return 0.0
    return float(np.median(vals))


class CameraVision(Node):
    def __init__(self, model_path='yolov8n.pt', conf=0.5, use_cuda=True, publish_image=True):
        super().__init__('camera_vision')
        self.get_logger().info('Starting RealSense YOLO node')

        # ROS publishers
        self.pub_detections = self.create_publisher(String, 'detections_json', 10)
        self.pub_point = self.create_publisher(PointStamped, 'detection_point', 10)
        self.publish_image = publish_image
        if self.publish_image:
            self.pub_image = self.create_publisher(Image, 'camera/color/image_raw', 10)
            self.bridge = CvBridge()

        # YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.device = 0 if (use_cuda and torch.cuda.is_available()) else 'cpu'
        self.get_logger().info(f'Using device: {self.device}')
        if self.device != 'cpu':
            self.model.to(f'cuda:{self.device}' if isinstance(self.device, int) else 'cuda')

        # Realsense pipeline setup
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        profile = self.pipeline.start(cfg)
        self.align_to_color = rs.align(rs.stream.color)

        # FPS smoothing
        self.fps_hist = deque(maxlen=20)
        self.conf_thres = conf

        # Run in a timer callback so node spins properly
        self.timer = self.create_timer(0.001, self.timer_cb)  # tiny timer: internal loop drives frames
        self._last_time = time.time()

    def timer_cb(self):
        # Run a single capture + inference
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        except Exception as e:
            self.get_logger().error(f"RealSense capture error: {e}")
            return

        aligned = self.align_to_color.process(frames)
        depth = aligned.get_depth_frame()
        color = aligned.get_color_frame()
        if not depth or not color:
            return

        color_img = np.asanyarray(color.get_data())

        # YOLO inference (fast)
        results = self.model.predict(
            source=color_img,
            conf=self.conf_thres,
            iou=0.45,
            device=0 if (self.device != 'cpu') else 'cpu',
            verbose=False
        )

        detections_out = []

        depth_intrin = depth.profile.as_video_stream_profile().get_intrinsics()

        for r in results:
            names = r.names
            for box in r.boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cls_id = int(box.cls[0].cpu().numpy())
                conf = float(box.conf[0].cpu().numpy())
                name = names.get(cls_id, str(cls_id))

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                dist_m = median_depth(depth, cx, cy, k=5)

                if dist_m > 0:
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [float(cx), float(cy)], dist_m)
                else:
                    X = 0.0
                    Y = 0.0
                    Z = 0.0

                # Compose detection dict
                det = {
                    'class_id': cls_id,
                    'class_name': name,
                    'confidence': float(conf),
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center_px': [int(cx), int(cy)],
                    'distance_m': float(dist_m),
                    'point_m': [float(X), float(Y), float(Z)]
                }
                detections_out.append(det)

                # Publish a PointStamped for this detection (frame 'camera_link' or color frame)
                pt_msg = PointStamped()
                pt_msg.header.stamp = self.get_clock().now().to_msg()
                pt_msg.header.frame_id = 'camera_link'  # set to your camera frame
                pt_msg.point.x = float(X)
                pt_msg.point.y = float(Y)
                pt_msg.point.z = float(Z)
                self.pub_point.publish(pt_msg)

                # Draw on image for display
                cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{name} {conf:.2f}"
                if dist_m > 0:
                    label += f" | {dist_m:.2f}m"
                cv2.putText(color_img, label, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish detections JSON
        if detections_out:
            j = json.dumps({'timestamp': time.time(), 'detections': detections_out})
            self.pub_detections.publish(String(data=j))

        # Publish image if requested
        if self.publish_image:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
                ros_img.header.stamp = self.get_clock().now().to_msg()
                ros_img.header.frame_id = 'camera_link'
                self.pub_image.publish(ros_img)
            except Exception as e:
                self.get_logger().warning(f"cv_bridge publish fail: {e}")

        # Update fps
        now = time.time()
        fps = 1.0 / max(1e-6, (now - self._last_time))
        self._last_time = now
        self.fps_hist.append(fps)
        if len(self.fps_hist) == self.fps_hist.maxlen:
            avg = sum(self.fps_hist) / len(self.fps_hist)
            self.get_logger().debug(f"YOLO FPS: {avg:.1f}")

    def destroy_node(self):
        '''
        Overwrite destroy_node function default
        '''
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraVision(model_path='yolov8n.pt', conf=0.5, use_cuda=True, publish_image=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
