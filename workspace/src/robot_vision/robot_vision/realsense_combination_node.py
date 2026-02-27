import json

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_vision.realsense_combination import (
    GroceryRobotVision,
    parse_host_port,
    parse_vec3_csv,
)


class RealsenseCombinationNode(Node):
    def __init__(self):
        super().__init__("realsense_combination_node")

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", 0.5)
        self.declare_parameter("use_imu", False)
        self.declare_parameter("cam_offset", "0,0,0")
        self.declare_parameter("cam_rpy", "0,0,0")
        self.declare_parameter("udp", "")
        self.declare_parameter("publish_image", True)
        self.declare_parameter("timer_period_s", 0.02)

        model = self.get_parameter("model").get_parameter_value().string_value
        conf = self.get_parameter("conf").get_parameter_value().double_value
        use_imu = self.get_parameter("use_imu").get_parameter_value().bool_value
        cam_offset = self.get_parameter("cam_offset").get_parameter_value().string_value
        cam_rpy = self.get_parameter("cam_rpy").get_parameter_value().string_value
        udp_raw = self.get_parameter("udp").get_parameter_value().string_value
        self.publish_image = self.get_parameter("publish_image").get_parameter_value().bool_value
        timer_period_s = self.get_parameter("timer_period_s").get_parameter_value().double_value

        cam_offset_base = parse_vec3_csv(cam_offset)
        cam_rpy_deg = parse_vec3_csv(cam_rpy)
        udp_target = parse_host_port(udp_raw) if udp_raw else None

        self.vision = GroceryRobotVision(
            yolo_model_path=model,
            conf_threshold=conf,
            use_imu=use_imu,
            cam_offset_base=cam_offset_base,
            cam_rpy_deg=cam_rpy_deg,
            udp_target=udp_target,
        )

        self.bridge = CvBridge()
        self.payload_pub = self.create_publisher(
            String, "realsense_combination/payload_json", 10
        )
        self.image_pub = None
        if self.publish_image:
            self.image_pub = self.create_publisher(
                Image, "realsense_combination/image_raw", 5
            )

        self.timer = self.create_timer(timer_period_s, self.timer_cb)
        self.get_logger().info("realsense_combination_node started")

    def timer_cb(self):
        color_img, products, barcodes, fps, imu_euler = self.vision.process_frame()
        if color_img is None:
            return

        payload = self.vision._build_payload(products, barcodes, fps, imu_euler)
        self.vision._send_udp(payload)

        self.payload_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        if self.image_pub is not None:
            display = self.vision.draw_results(
                color_img.copy(), products, barcodes, fps, imu_euler
            )
            ros_img = self.bridge.cv2_to_imgmsg(display, encoding="bgr8")
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "camera_link"
            self.image_pub.publish(ros_img)

    def destroy_node(self):
        try:
            self.vision.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCombinationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

