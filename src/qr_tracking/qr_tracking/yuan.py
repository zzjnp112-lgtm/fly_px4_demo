#!/usr/bin/env python3
# 视觉追踪节点：订阅仿真相机图像，检测二维码，发布二维码中心像素坐标到 /zzjnp
# 输入：image_topic (默认 /drone_0_/camera/image_raw，下视 gemini2)
# 输出：/zzjnp (geometry_msgs/Point, 像素坐标) + /circle_color (std_msgs/Float64)

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge


class QrNode(Node):
    def __init__(self):
        super().__init__('qr_node')

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # 图像 topic 参数（下视 gemini2 / 前视 front_cam 可切换）
        self.declare_parameter('image_topic', '/drone_0_/camera/image_raw')
        image_topic = self.get_parameter('image_topic').value

        # 发布：二维码中心像素坐标
        self.center_pub = self.create_publisher(Point, 'zzjnp', 10)
        self.pub_color = self.create_publisher(Float64, 'circle_color', 10)

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # 无目标时的默认中心（图像中心 640x480）
        self.default_center = Point(x=320.0, y=240.0, z=0.0)

        self.get_logger().info(f'qr_node 启动，订阅 {image_topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        # 检测并解码二维码
        data, bbox, _ = self.detector.detectAndDecode(frame)

        if data and bbox is not None and len(bbox) > 0:
            # bbox 是 4 个角点，取中心
            pts = np.asarray(bbox).reshape(-1, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            self.center_pub.publish(Point(x=cx, y=cy, z=0.0))
            self.pub_color.publish(Float64(data=1.0))
            self.get_logger().info(
                f'检测到二维码: "{data}", 中心=({cx:.0f}, {cy:.0f})',
                throttle_duration_sec=1.0)
        else:
            # 无二维码：发布默认中心（图像中心）
            self.center_pub.publish(self.default_center)


def main(args=None):
    rclpy.init(args=args)
    node = QrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
