#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import random
import math
from interfaces.srv import RoiToPoint, RoiToPointResponse
from geometry_msgs.msg import Point

class stereoCamera(object):
    def __init__(self):
        # 左相机内参
        self.cam_matrix_left = np.array([[519.986208891309, 0.406696263103782, 322.800493646764],[0, 520.076546808808, 240.037862316179],[0.,0.,1.]])
        # 右相机内参
        self.cam_matrix_right = np.array([[514.941499937349, 0.336532156527875, 291.833723212528],[0, 514.874180707603, 241.201518688453],[0.,0.,1.]])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = np.array([[-0.054568256319688, 0.365710553696214, -0.00244714704429, -0.001503496381615]])#-0.778416903681303
        self.distortion_r = np.array([[-0.067526549493682, 0.420999197085466, -0.001590771322507, -0.001741268417658]])#-0.679200259626978

        # 旋转矩阵
        self.R = np.array([[0.999989550908624, 0.0011552556161, 0.004423059803995],
                        [-0.001145008514507, 0.999996656573218, -0.00231857669421],
                        [-0.004425723564566, 0.002313488026054, 0.999987530294295]])


        # 平移矩阵
        self.T = np.array([-58.2697787032485, -0.212211320328094, 1.20710537108232])
        self.size = (640, 480)


class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        rospy.loginfo("节点已启动: camera_publisher!")
        self.img_width = 1280
        self.img_height = 480      

        self.fps = 0.0
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            rospy.loginfo("无法打开摄像头")
            return
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        
        self.bridge = CvBridge()
        self.sterea_camear = stereoCamera()
        self.publisher_image = rospy.Publisher('camera_image', Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.roi_to_point_serve = rospy.Service('roi_to_point_serve', RoiToPoint, self.roi_to_point_callback)

    def timer_callback(self, event):
        ret, frame = self.cap.read()
        if ret:
            self.frame1 = frame[0:480, 0:640]
            self.frame2 = frame[0:480, 640:1280]
            # 将调整大小后的图像转换为ROS图像消息并发布
            color_frame = self.bridge.cv2_to_imgmsg(self.frame1, encoding='bgr8')
            self.publisher_image.publish(color_frame)
        else:       
            rospy.loginfo("color_frame无法读取摄像头帧")

    def roi_to_point_callback(self, request): 
        roi = request.roi
        mid_x = int(roi.x_offset +roi.width*0.5)
        mid_y = int(roi.y_offset +roi.height *0.5)

        ret, frame = self.cap.read()
        if ret:
            target = self.sgbm(frame, mid_x, mid_y)
            target = target
        else:
            target = Point()
        response = RoiToPointResponse()
        response.target = target
        return response

    def sgbm(self, frame, x, y):
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(self.sterea_camear.cam_matrix_left, self.sterea_camear.distortion_l,
                                                                        self.sterea_camear.cam_matrix_right, self.sterea_camear.distortion_r, 
                                                                        self.sterea_camear.size, 
                                                                        self.sterea_camear.R, self.sterea_camear.T,flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1)

        left_map1, left_map2 = cv2.initUndistortRectifyMap(self.sterea_camear.cam_matrix_left, self.sterea_camear.distortion_l, R1, P1, self.sterea_camear.size, cv2.CV_16SC2)
        right_map1, right_map2 = cv2.initUndistortRectifyMap(self.sterea_camear.cam_matrix_right, self.sterea_camear.distortion_r, R2, P2, self.sterea_camear.size, cv2.CV_16SC2)
    
        frame1 = frame[0:480, 0:640]
        frame2 = frame[0:480, 640:1280]

        imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        img1_rectified = cv2.remap(imgL, left_map1, left_map2, cv2.INTER_LINEAR)
        img2_rectified = cv2.remap(imgR, right_map1, right_map2, cv2.INTER_LINEAR)
        # ------------------------------------SGBM算法----------------------------------------------------------
        #   blockSize                   深度图成块，blocksize越低，其深度图就越零碎，0<blockSize<10
        #   img_channels                BGR图像的颜色通道，img_channels=3，不可更改
        #   numDisparities              SGBM感知的范围，越大生成的精度越好，速度越慢，需要被16整除，如numDisparities
        #                               取16、32、48、64等
        #   mode                        sgbm算法选择模式，以速度由快到慢为：STEREO_SGBM_MODE_SGBM_3WAY、
        #                               STEREO_SGBM_MODE_HH4、STEREO_SGBM_MODE_SGBM、STEREO_SGBM_MODE_HH。精度反之
        # ------------------------------------------------------------------------------------------------------
        blockSize = 8
        numDisparities = 16 * 5  # 必须是16的倍数
        stereo = cv2.StereoSGBM_create(minDisparity=0,
                                    numDisparities=numDisparities,
                                    blockSize=blockSize,
                                    P1=8 * blockSize * blockSize,
                                    P2=32 * blockSize * blockSize,
                                    disp12MaxDiff=1,
                                    preFilterCap=1,
                                    uniquenessRatio=10,
                                    speckleWindowSize=100,
                                    speckleRange=100,
                                    mode=cv2.STEREO_SGBM_MODE_HH)
        # 计算视差
        disparity = stereo.compute(img1_rectified, img2_rectified).astype(np.float32) / 16.0
        disparity = cv2.medianBlur(disparity, 5)
        points_3d = cv2.reprojectImageTo3D(disparity, Q,handleMissingValues=True)
        rep_point = Point()
        rep_point.x, rep_point.y, rep_point.z = points_3d[y, x, 0] / 10, points_3d[y, x, 1] / 10, points_3d[y, x, 2] / 10

        return rep_point

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CameraPublisher()
    node.run()
