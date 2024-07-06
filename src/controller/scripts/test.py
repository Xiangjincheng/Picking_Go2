#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        rospy.loginfo("节点已启动: camera_publisher!")
        self.img_width = 640
        self.img_height = 480

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            rospy.logerr("无法打开摄像头")
            return
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)

        self.bridge = CvBridge()
        self.publisher_image = rospy.Publisher('camera_image', Image, queue_size=10)
        # self.publisher_depth = rospy.Publisher('camera_depth', Range, queue_size=10)  # 创建深度信息发布者

        # 设置内参矩阵和畸变系数
        self.camera_matrix = np.array(  [[1.74192606e+03, 0.00000000e+00, 2.94158188e+02],
                                        [0.00000000e+00, 1.51257698e+03, 1.65678778e+02],
                                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)
        self.dist_coeffs = np.array([4.83697282e-03, -5.64171037e+01, 2.06721865e-02, 7.13074916e-03, 2.35169075e+03], dtype=np.float32)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        # self.K = 5000  # 测距算法中使用的常数，需要根据实际情况调整
        
    def timer_callback(self, event):
        ret, frame = self.cap.read()

        if ret:
             # 校正图像畸变
            undistorted_frame = self.undistort_image(frame)
    
            # # 进行测距算法
            # blobs = self.detect_object(resized_frame)
            # if len(blobs) == 1:
            #     b = blobs[0]
            #     Lm = (b[2] + b[3]) / 2
            #     depth_value = self.K / Lm
            #     # 创建并发布ROS Range消息
            #     depth_msg = Range()
            #     depth_msg.header.stamp = rospy.Time.now()
            #     depth_msg.header.frame_id = 'camera_depth'
            #     depth_msg.radiation_type = Range.INFRARED
            #     depth_msg.field_of_view = 0.1  # 视场角，需要根据实际情况调整
            #     depth_msg.min_range = 0.1  # 最小测量范围
            #     depth_msg.max_range = 10.0  # 最大测量范围
            #     depth_msg.range = depth_value  # 深度值
            #     self.publisher_depth.publish(depth_msg)

            # # 将调整大小后的图像转换为ROS图像消息并发布
            color_frame = self.bridge.cv2_to_imgmsg(undistorted_frame, encoding='bgr8')
            self.publisher_image.publish(color_frame)
        else:       
            rospy.logerr("无法读取摄像头帧")

        color_frame = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_image.publish(color_frame)


    def undistort_image(self, frame):
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix)
        return undistorted_frame
    
    # def detect_object(self, frame):
    #     # 在这里实现你的物体检测算法，返回检测到的斑点信息
    #     yellow_lower = np.array([56, 83, 5])
    #     yellow_upper = np.array([57, 63, 80])
    #     mask = cv2.inRange(frame, yellow_lower, yellow_upper)
    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    #     blobs = []
    #     for contour in contours:
    #         x, y, w, h = cv2.boundingRect(contour)
    #         blobs.append((x, y, w, h))
        
    #     return blobs

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CameraPublisher()
    node.run()
