#!/usr/bin/env python3

import sys
from threading import Thread
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform
from tf.transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, quaternion_inverse

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor
from PyQt5.QtCore import Qt, QPoint, QTimer, QRect

from franka_pickup import Manipulator


def inverse_transform(transform):
    # 提取平移和旋转
    translation = np.array([transform.translation.x, 
                           transform.translation.y, 
                           transform.translation.z])
    rotation = np.array([transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z,
                        transform.rotation.w])
    
    # 构造齐次变换矩阵
    rot_matrix = quaternion_matrix(rotation)
    rot_matrix[:3, 3] = translation
    
    # 求逆矩阵
    inv_matrix = np.linalg.inv(rot_matrix)
    
    # 提取逆变换的平移和旋转
    inv_translation = translation_from_matrix(inv_matrix)
    inv_rotation = quaternion_from_matrix(inv_matrix)
    
    # 创建新的Transform对象
    inv_transform = Transform()
    inv_transform.translation.x = inv_translation[0]
    inv_transform.translation.y = inv_translation[1]
    inv_transform.translation.z = inv_translation[2]
    inv_transform.rotation.x = inv_rotation[0]
    inv_transform.rotation.y = inv_rotation[1]
    inv_transform.rotation.z = inv_rotation[2]
    inv_transform.rotation.w = inv_rotation[3]
    
    return inv_transform

def get_R_T(transform):
    T = np.array([transform.translation.x, 
                    transform.translation.y, 
                    transform.translation.z])
    quat = [transform.rotation.x, 
                transform.rotation.y, 
                transform.rotation.z, 
                transform.rotation.w]
    R = quaternion_matrix(quat)[:3, :3]
    return R, T 

class ImageDisplayWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.image = None
        self.original_image_size = None  # 存储原始图像尺寸
        self.display_rect = None  # 存储当前显示区域
        self.points = []  # 存储原始图像坐标点
        self.mode = None
        
    def set_image(self, cv_image):
        """设置要显示的OpenCV图像"""
        if cv_image is not None:
            self.original_image_size = (cv_image.shape[1], cv_image.shape[0])  # (width, height)
            if len(cv_image.shape) == 2:  # 灰度图
                qimage = QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], 
                               cv_image.shape[1], QImage.Format_Grayscale8)
            else:  # 彩色图
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                qimage = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], 
                               rgb_image.shape[1] * 3, QImage.Format_RGB888)
            self.image = QPixmap.fromImage(qimage)
            self.update()
    
    def set_mode(self, mode):
        """设置当前交互模式"""
        self.mode = mode
        assert mode in ['flatten', 'fold', 'dual_fold', 'random']
        self.points = []
        self.update()
    
    def mousePressEvent(self, event):
        """鼠标点击事件处理"""
        if event.button() == Qt.LeftButton and self.mode in ['flatten', 'fold', 'dual_fold', 'random']:
            if self.display_rect and self.original_image_size:
                # 将窗口坐标转换为原始图像坐标
                x = (event.pos().x() - self.display_rect.x()) * self.original_image_size[0] / self.display_rect.width()
                y = (event.pos().y() - self.display_rect.y()) * self.original_image_size[1] / self.display_rect.height()
                
                if 0 <= x < self.original_image_size[0] and 0 <= y < self.original_image_size[1]:
                    if self.mode == 'flatten' and len(self.points) < 2:
                        self.points.append(QPoint(int(x), int(y)))
                    elif self.mode == 'fold' and len(self.points) < 2:
                        self.points.append(QPoint(int(x), int(y)))
                    elif self.mode == 'dual_fold' and len(self.points) < 4:
                        self.points.append(QPoint(int(x), int(y)))
                    elif self.mode == 'random' and len(self.points) < 1:
                        self.points.append(QPoint(int(x), int(y)))
                    self.update()
    
    def paintEvent(self, event):
        """绘制图像和交互元素"""
        painter = QPainter(self)
        
        # 计算显示区域(保持宽高比)
        if self.image and self.original_image_size:
            img_ratio = self.original_image_size[0] / self.original_image_size[1]
            win_ratio = self.width() / self.height()
            
            if img_ratio > win_ratio:
                # 以宽度为准
                display_width = self.width()
                display_height = int(display_width / img_ratio)
                x = 0
                y = (self.height() - display_height) // 2
            else:
                # 以高度为准
                display_height = self.height()
                display_width = int(display_height * img_ratio)
                x = (self.width() - display_width) // 2
                y = 0
                
            self.display_rect = QRect(x, y, display_width, display_height)
            painter.drawPixmap(self.display_rect, self.image)
            
            # 绘制交互元素(将原始坐标转换为显示坐标)
            scale_x = self.display_rect.width() / self.original_image_size[0]
            scale_y = self.display_rect.height() / self.original_image_size[1]
            
            if self.mode == 'flatten' and self.points:
                painter.setPen(QPen(QColor(0, 0, 255), 5))
                for point in self.points:
                    display_x = self.display_rect.x() + point.x() * scale_x
                    display_y = self.display_rect.y() + point.y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)
            
            elif self.mode in ['fold', 'dual_fold'] and self.points:
                if len(self.points) >= 1:
                    painter.setPen(QPen(QColor(0, 0, 255), 5))
                    display_x = self.display_rect.x() + self.points[0].x() * scale_x
                    display_y = self.display_rect.y() + self.points[0].y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)
                
                if len(self.points) >= 2:
                    painter.setPen(QPen(QColor(255, 0, 0), 5))
                    display_x = self.display_rect.x() + self.points[1].x() * scale_x
                    display_y = self.display_rect.y() + self.points[1].y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)
                    
                    # 绘制虚线连接线
                    pen = QPen(QColor(0, 255, 0), 2, Qt.DashLine)
                    painter.setPen(pen)
                    start_x = self.display_rect.x() + self.points[0].x() * scale_x
                    start_y = self.display_rect.y() + self.points[0].y() * scale_y
                    end_x = self.display_rect.x() + self.points[1].x() * scale_x
                    end_y = self.display_rect.y() + self.points[1].y() * scale_y
                    painter.drawLine(QPoint(int(start_x), int(start_y)), 
                                   QPoint(int(end_x), int(end_y)))

                if len(self.points) >= 3:
                    painter.setPen(QPen(QColor(0, 0, 255), 5))
                    display_x = self.display_rect.x() + self.points[2].x() * scale_x
                    display_y = self.display_rect.y() + self.points[2].y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)

                if len(self.points) >= 4:
                    painter.setPen(QPen(QColor(255, 0, 0), 5))
                    display_x = self.display_rect.x() + self.points[3].x() * scale_x
                    display_y = self.display_rect.y() + self.points[3].y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)
                    
                    # 绘制虚线连接线
                    pen = QPen(QColor(0, 255, 0), 2, Qt.DashLine)
                    painter.setPen(pen)
                    start_x = self.display_rect.x() + self.points[2].x() * scale_x
                    start_y = self.display_rect.y() + self.points[2].y() * scale_y
                    end_x = self.display_rect.x() + self.points[3].x() * scale_x
                    end_y = self.display_rect.y() + self.points[3].y() * scale_y
                    painter.drawLine(QPoint(int(start_x), int(start_y)), 
                                   QPoint(int(end_x), int(end_y)))
            elif self.mode == 'random' and self.points:
                if len(self.points) >= 1:
                    painter.setPen(QPen(QColor(0, 0, 255), 5))
                    display_x = self.display_rect.x() + self.points[0].x() * scale_x
                    display_y = self.display_rect.y() + self.points[0].y() * scale_y
                    painter.drawEllipse(QPoint(int(display_x), int(display_y)), 5, 5)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Image Viewer with Interaction")
        self.setGeometry(100, 100, 1200, 600)
        
        # ROS初始化
        self.bridge = CvBridge()
        self.cv_rgb_image = None
        self.cv_depth_image = None
        self.camera_info = None
        self.left_arm_pose = None
        self.right_arm_pose = None
        self.camera_pose = None
        
        # 主界面布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)
        
        # 左侧图像显示区域
        self.image_display = ImageDisplayWidget()
        layout.addWidget(self.image_display, 3)
        
        # 右侧控制区域
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        layout.addWidget(control_panel, 1)
        
        # 添加按钮
        self.flatten_btn = QPushButton("铺平")
        self.flatten_btn.clicked.connect(lambda: self.image_display.set_mode('flatten'))
        control_layout.addWidget(self.flatten_btn)
        
        self.fold_btn = QPushButton("折叠")
        self.fold_btn.clicked.connect(lambda: self.image_display.set_mode('fold'))
        control_layout.addWidget(self.fold_btn)
        
        self.dual_fold_btn = QPushButton("对折")
        self.dual_fold_btn.clicked.connect(lambda: self.image_display.set_mode('dual_fold'))
        control_layout.addWidget(self.dual_fold_btn)
        
        self.start_btn = QPushButton("开始")
        self.start_btn.clicked.connect(self.start_processing)
        control_layout.addWidget(self.start_btn)
        
        self.reset_btn = QPushButton("重置")
        self.reset_btn.clicked.connect(lambda: self.image_display.set_mode('reset'))
        control_layout.addWidget(self.reset_btn)
        
        self.random_btn = QPushButton("随机")
        self.random_btn.clicked.connect(lambda: self.image_display.set_mode('random'))
        control_layout.addWidget(self.random_btn)

        # 状态标签
        self.status_label = QLabel("状态: 等待输入")
        control_layout.addWidget(self.status_label)
        
        # 添加伸缩空间使按钮靠上
        control_layout.addStretch()
        
        # ROS订阅器
        self.rgb_sub = rospy.Subscriber("/rgb", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/depth", Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber("/camera_info", CameraInfo, self.camera_info_callback)
        self.left_arm_pose_sub = rospy.Subscriber("/panda_left/pose_in_world", TFMessage, self.left_arm_pose_callback)
        self.right_arm_pose_sub = rospy.Subscriber("/panda_right/pose_in_world", TFMessage, self.right_arm_pose_callback)
        self.camera_pose_sub = rospy.Subscriber("/camera/pose_in_world", TFMessage, self.camera_pose_callback)
        
        # 定时器更新图像显示
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(30)  # 约30fps

        # 机械臂
        self.panda_left = Manipulator("panda_left")
        self.panda_left.move_to_home()
        self.panda_left.open_gripper()
        self.panda_right = Manipulator("panda_right")
        self.panda_right.move_to_home()
        self.panda_right.open_gripper()
    
    def rgb_callback(self, msg):
        """RGB图像回调"""
        try:
            self.cv_rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def camera_info_callback(self, msg):
        """相机参数回调"""
        self.camera_info = msg
    
    def left_arm_pose_callback(self, msg):
        """左机械臂位姿回调"""
        for transform in msg.transforms:
            if transform.child_frame_id == "panda_left":  # 根据实际frame_id调整
                self.left_arm_pose = transform.transform
                break

    def right_arm_pose_callback(self, msg):
        """右机械臂位姿回调"""
        for transform in msg.transforms:
            if transform.child_frame_id == "panda_right":  # 根据实际frame_id调整
                self.right_arm_pose = transform.transform
                break

    def camera_pose_callback(self, msg):
        """相机位姿回调"""
        for transform in msg.transforms:
            if transform.child_frame_id == "camera":  # 根据实际frame_id调整
                self.camera_pose = transform.transform
                break
    
    def update_display(self):
        """更新图像显示"""
        if self.cv_rgb_image is not None:
            self.image_display.set_image(self.cv_rgb_image)
    
    def start_processing(self):
        """开始处理按钮点击事件"""
        # 检查数据是否完整
        if not self.check_data_ready():
            self.status_label.setText("状态: 数据不完整，无法处理")
            return
        
        # 获取图像上的点并转换为原始图像坐标
        points = [(p.x(), p.y()) for p in self.image_display.points]
        
        # 计算世界坐标
        world_points = self.calculate_world_coordinates(points)
        
        # 显示结果
        if len(world_points) == 2:
            self.status_label.setText(f"状态: 计算完成\n点1: {world_points[0]}\n点2: {world_points[1]}")
        elif len(world_points) == 4:
            self.status_label.setText(f"状态: 计算完成\n点1: {world_points[0]}\n点2: {world_points[1]}\n点3: {world_points[2]}\n点4: {world_points[3]}")
        elif len(world_points) == 1:
            self.status_label.setText(f"状态: 计算完成\n点: {world_points[0]}")

        if self.image_display.mode == "fold":
            if world_points[0][0] < 0:
                arm = self.panda_left
                R, T = get_R_T(inverse_transform(self.left_arm_pose))
            else:
                arm = self.panda_right
                R, T = get_R_T(inverse_transform(self.right_arm_pose))
            pick_point = R.dot(world_points[0]) + T
            place_point = R.dot(world_points[1]) + T
            Thread(target=arm.pick_and_place, args=(pick_point, place_point)).start()
        elif self.image_display.mode == "dual_fold":
            if world_points[0][0] < world_points[1][0]:
                arm1 = self.panda_left
                R1, T1 = get_R_T(inverse_transform(self.left_arm_pose))
                arm2 = self.panda_right
                R2, T2 = get_R_T(inverse_transform(self.right_arm_pose))
            else:
                arm1 = self.panda_right
                R1, T1 = get_R_T(inverse_transform(self.right_arm_pose))
                arm2 = self.panda_left
                R2, T2 = get_R_T(inverse_transform(self.left_arm_pose))
            pick_point1 = R1.dot(world_points[0]) + T1
            place_point1 = R1.dot(world_points[1]) + T1
            pick_point2 = R2.dot(world_points[2]) + T2
            place_point2 = R2.dot(world_points[3]) + T2
            Thread(target=arm1.pick_and_place, args=(pick_point1, place_point1, True, True)).start()
            Thread(target=arm2.pick_and_place, args=(pick_point2, place_point2, True, True)).start()
        elif self.image_display.mode == "flatten":
            if world_points[0][0] < world_points[1][0]:
                arm1 = self.panda_left
                R1, T1 = get_R_T(inverse_transform(self.left_arm_pose))
                arm2 = self.panda_right
                R2, T2 = get_R_T(inverse_transform(self.right_arm_pose))
            else:
                arm1 = self.panda_right
                R1, T1 = get_R_T(inverse_transform(self.right_arm_pose))
                arm2 = self.panda_left
                R2, T2 = get_R_T(inverse_transform(self.left_arm_pose))
            pick_point1 = R1.dot(world_points[0]) + T1
            pick_point2 = R2.dot(world_points[1]) + T2
            Thread(target=arm1.flatten, args=(pick_point1, True)).start()
            Thread(target=arm2.flatten, args=(pick_point2, True)).start()
        elif self.image_display.mode == "random":
            if world_points[0][0] < 0:
                arm = self.panda_left
                R, T = get_R_T(inverse_transform(self.left_arm_pose))
            else:
                arm = self.panda_right
                R, T = get_R_T(inverse_transform(self.right_arm_pose))
            pick_point = R.dot(world_points[0]) + T
            Thread(target=arm.pick_and_drop, args=(pick_point, )).start()

    
    def check_data_ready(self):
        """检查所需数据是否已准备好"""
        return (self.cv_rgb_image is not None and 
                self.cv_depth_image is not None and 
                self.camera_info is not None and 
                self.left_arm_pose is not None and 
                self.right_arm_pose is not None and 
                self.camera_pose is not None)
    
    def calculate_world_coordinates(self, img_points):
        """计算图像点在世界坐标系中的坐标"""
        # 这里实现从图像坐标到世界坐标的转换
        # 需要相机内参、深度图和相机位姿
        
        # 1. 获取相机内参
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        
        # 2. 获取相机位姿 (从Transform消息转换为4x4变换矩阵)
        # TODO: isaac sim中给的相机坐标有问题
        self.camera_pose.rotation.x = 1.0
        self.camera_pose.rotation.y = 0.0
        self.camera_pose.rotation.z = 0.0
        self.camera_pose.rotation.w = 0.0
        R, T = get_R_T(self.camera_pose)
        
        # 3. 对于每个图像点，计算其世界坐标
        world_points = []
        for u, v in img_points:
            # 确保坐标在图像范围内
            u = int(np.clip(u, 0, self.cv_depth_image.shape[1]-1))
            v = int(np.clip(v, 0, self.cv_depth_image.shape[0]-1))
            
            # 获取深度值
            depth = self.cv_depth_image[v, u]
            
            # 从图像坐标到相机坐标
            x = (u - cx) * depth / fx
            y = (v - cy) * depth / fy
            z = depth
            
            # 相机坐标到世界坐标
            camera_coord = np.array([x, y, z])
            world_coord = R.dot(camera_coord) + T
            
            world_points.append(world_coord)
        
        return world_points


def main():
    rospy.init_node('image_interaction_node', anonymous=True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()