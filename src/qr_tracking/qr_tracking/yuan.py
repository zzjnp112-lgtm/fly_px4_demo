#!/usr/bin/env python3
# 视觉追踪节点：订阅仿真相机图像，检测二维码，卡尔曼滤波后发布二维码中心像素坐标到 /zzjnp
# 输入：image_topic (默认 /drone_0_/camera/image_raw，下视 gemini2)
# 输出：/zzjnp (geometry_msgs/Point, 滤波后像素坐标) + /circle_color (std_msgs/Float64, 1=检测到 0=丢失预测)

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from cv_bridge import CvBridge


class KalmanFilter2D:
    """匀速模型卡尔曼滤波：状态 [x, y, vx, vy]，测量 [x, y]。

    用途：平滑二维码检测抖动，并在二维码短暂丢失时用预测位置继续输出。
    """
    def __init__(self, dt, q_pos=0.05, q_vel=0.5, r_meas=3.0):
        self.dt = dt
        # 状态转移矩阵（匀速模型）：位置 += 速度*dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]], np.float64)
        # 测量矩阵：只观测位置
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]], np.float64)
        # 过程噪声协方差（模型不确定性）
        self.Q = np.eye(4, dtype=np.float64)
        self.Q[0, 0] = self.Q[1, 1] = q_pos
        self.Q[2, 2] = self.Q[3, 3] = q_vel
        # 测量噪声协方差（检测噪声）
        self.R = np.eye(2, dtype=np.float64) * r_meas
        # 状态和协方差（初始不确定度设大，第一次检测会自动收敛）
        self.x = np.zeros((4, 1), np.float64)
        self.P = np.eye(4, dtype=np.float64) * 1000.0
        self.initialized = False

    def predict(self):
        """预测一步，返回预测后的位置 (x, y)。"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0, 0], self.x[1, 0]

    def correct(self, mx, my):
        """用测量 (mx, my) 校正，返回滤波后的位置 (x, y)。"""
        z = np.array([[mx], [my]], np.float64)
        y = z - self.H @ self.x          # 创新（测量残差）
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)   # 卡尔曼增益
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.initialized = True
        return self.x[0, 0], self.x[1, 0]

    def pos(self):
        """当前（预测后的）位置 (x, y)。"""
        return self.x[0, 0], self.x[1, 0]

    def decay_velocity(self, factor):
        """速度按 factor 衰减（丢失时让预测逐渐停下，防止外推发散）。"""
        self.x[2, 0] *= factor
        self.x[3, 0] *= factor

    def reset(self, mx, my):
        """重置滤波器到 (mx, my)：速度清零，协方差重新初始化。"""
        self.x = np.zeros((4, 1), np.float64)
        self.x[0, 0] = mx
        self.x[1, 0] = my
        self.P = np.eye(4, dtype=np.float64) * 1000.0
        self.initialized = True


class QrNode(Node):
    def __init__(self):
        super().__init__('qr_node')

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # 图像 topic 参数（下视 gemini2 / 前视 front_cam 可切换）
        self.declare_parameter('image_topic', '/drone_0_/camera/image_raw')
        image_topic = self.get_parameter('image_topic').value

        self.center_pub = self.create_publisher(Point, 'zzjnp', 10)
        self.pub_color = self.create_publisher(Float64, 'circle_color', 10)
        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # 无目标时的默认中心（图像中心 1280x720）
        self.default_center = Point(x=640.0, y=360.0, z=0.0)

        # 卡尔曼滤波参数：
        #   dt     = 1/30（约30Hz 相机帧间隔）
        #   q_pos  = 位置过程噪声，调大 → 更贴测量（抖动大），调小 → 更平滑
        #   q_vel  = 速度过程噪声，调大 → 对目标速度变化反应更快（适合乱走），调小 → 更平滑
        #   r_meas = 测量噪声，调大 → 更平滑（更信任模型），调小 → 更贴检测值
        self.kf = KalmanFilter2D(dt=1.0/30.0, q_pos=0.03, q_vel=1.3, r_meas=1.2)
        self.lost_count = 0  # 连续丢失帧数

        self.get_logger().info(f'qr_node 启动（含卡尔曼滤波），订阅 {image_topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}')
            return

        # 只检测二维码位置（不解码：容器里 OpenCV 没链接 QUIRC）
        retval, points = self.detector.detect(frame)

        if retval and points is not None and len(points) > 0:
            # 检测到
            pts = np.asarray(points).reshape(-1, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            if self.lost_count > 0:
                # 之前丢失过，重新识别到 → 重置滤波器
                self.kf.reset(cx, cy)
                fx, fy = cx, cy
            else:
                # 连续检测 → 正常预测+校正
                self.kf.predict()
                fx, fy = self.kf.correct(cx, cy)

            self.lost_count = 0
            fx = min(max(fx, 0.0), 1280.0)
            fy = min(max(fy, 0.0), 720.0)

            self.center_pub.publish(Point(x=float(fx), y=float(fy), z=0.0))
            self.pub_color.publish(Float64(data=1.0))
            self.get_logger().info(
                f'检测到二维码 原始=({cx:.0f},{cy:.0f}) 滤波=({fx:.0f},{fy:.0f})',
                throttle_duration_sec=1.0)

            # 可视化：画边框 + 滤波后中心（红色）
            pts_i = pts.astype(int)
            cv2.polylines(frame, [pts_i], True, (0, 255, 0), 2)
            cv2.circle(frame, (int(fx), int(fy)), 5, (0, 0, 255), -1)
        else:
            # 无检测
            self.lost_count += 1
            if not self.kf.initialized:
                # 从未检测到过：用默认中心
                self.center_pub.publish(self.default_center)
            elif self.lost_count <= 60:
                # 丢失 2 秒内：直接预测（允许小幅发散，2秒后就不再预测了）
                self.kf.predict()
                px, py = self.kf.pos()
                self.center_pub.publish(Point(x=float(px), y=float(py), z=0.0))
                self.pub_color.publish(Float64(data=0.0))
                cv2.circle(frame, (int(px), int(py)), 5, (0, 165, 255), -1)  # 橙色=预测
            else:
                # 丢失超过 2 秒：发图像中心点（让无人机回到中心悬停）
                self.center_pub.publish(self.default_center)
                self.pub_color.publish(Float64(data=0.0))

        # 显示结果
        try:
            cv2.imshow('QR Detection', frame)
            cv2.waitKey(1)
        except Exception:
            pass


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
