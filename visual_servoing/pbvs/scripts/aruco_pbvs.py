#!/usr/bin/env python3
"""
ArUco-based Position-Based Visual Servoing (PBVS) for 6-DoF Robot
作者：香港时间 2025-11-17 17:10 为你定制
适用：xMate、Franka、UR、ABB 等所有 6/7 轴机械臂
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArucoPBVSNode(Node):
    def __init__(self):
        super().__init__('aruco_pbvs_node')
        
        # ==================== 参数 ====================
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('marker_id', 0)           # 要追踪的 ArUco ID
        self.declare_parameter('marker_size', 0.05)      # 单位：米（5cm）
        self.declare_parameter('velocity_topic', '/servo_server/delta_twist_cmds')
        
        camera_topic = self.get_parameter('camera_topic').value
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        self.vel_topic = self.get_parameter('velocity_topic').value

        # 相机内参（请替换成你自己的！）
        self.K = np.array([[615.456, 0.0, 320.5],
                           [0.0, 615.456, 240.5],
                           [0.0, 0.0, 1.0]], dtype=np.float64)

        # ArUco 设置
        # TODO AuUco字典需要设置
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        # 订阅 & 发布
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, 10)
        self.vel_pub = self.create_publisher(TwistStamped, self.vel_topic, 10)

        # 控制增益（调参神器）
        self.lambda_gain = np.diag([0.5, 0.5, 0.5, 1.2, 1.2, 1.2])  # 位置+姿态增益

        self.get_logger().info(f'ArUco PBVS Node Started! Tracking Marker ID: {self.marker_id}')
        self.get_logger().info(f'Camera: {camera_topic} | Vel: {self.vel_topic}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
        # cv_image 原始图像
        # 检测 ArUco
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None or self.marker_id not in ids:
            self.get_logger().warn_throttle(2.0, f"Marker {self.marker_id} not detected!")
            return

        idx = np.where(ids == self.marker_id)[0][0]
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[idx], self.marker_size, self.K, None)

        rvec = rvec[0][0]
        tvec = tvec[0][0]

        # 构建当前位姿（相机坐标系下的目标位姿）
        R_cam_to_marker, _ = cv2.Rodrigues(rvec)
        t_cam_to_marker = tvec

        # 期望位姿（我们希望目标在相机正前方 0.5m，Z 轴对齐）
        t_desired = np.array([0.0, 0.0, 0.50])  # 0.5m 正前方
        R_desired = np.eye(3)  # 期望姿态与相机对齐

        # 误差
        error_t = t_desired - t_cam_to_marker
        error_R = R_desired @ R_cam_to_marker.T
        error_rvec, _ = cv2.Rodrigues(error_R)
        error_rvec = error_rvec.flatten()

        # PBVS 控制律：v_c = -λ Λ^+ s_error
        s_error = np.hstack([error_t, error_rvec[:3]])  # 6维误差
        v_c = -self.lambda_gain @ s_error

        # 发布速度指令（相机坐标系）
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "camera_link"
        twist.twist.linear.x = v_c[0]
        twist.twist.linear.y = v_c[1]
        twist.twist.linear.z = v_c[2]
        twist.twist.angular.x = v_c[3]
        twist.twist.angular.y = v_c[4]
        twist.twist.angular.z = v_c[5]

        self.vel_pub.publish(twist)

        # 可视化（可选）
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        cv2.drawFrameAxes(cv_image, self.K, None, rvec, tvec, self.marker_size * 1.5)
        cv2.putText(cv_image, f"ID: {self.marker_id}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("ArUco PBVS", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPBVSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()