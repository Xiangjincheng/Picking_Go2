#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import math
from interfaces.srv import Depth, DepthResponse
from geometry_msgs.msg import Point

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
        self.publisher_image = rospy.Publisher('camera_image', Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.depth_serve = rospy.Service('depth_serve', Depth, self.depth_callback)

    def timer_callback(self, event):
        ret, frame = self.cap.read()
        self.frame1 = frame[0:480, 0:640]
        self.frame2 = frame[0:480, 640:1280]
        if ret:
            # 将调整大小后的图像转换为ROS图像消息并发布
            color_frame = self.bridge.cv2_to_imgmsg(self.frame1, encoding='bgr8')
            self.publisher_image.publish(color_frame)
        else:       
            rospy.loginfo("color_frame无法读取摄像头帧")
    def depth_callback(self, request): 
        roi = request.roi
        mid_x = int(roi.x_offset +roi.width*0.5)
        mid_y = int(roi.y_offset +roi.height *0.5)

        ret, frame = self.cap.read()
        if ret:
            rep_point = self.sgbm(frame, mid_x, mid_y)
        else:
            rep_point = Point()
            rospy.loginfo("无法读取摄像头帧")
        return DepthResponse(rep_point)

    def sgbm(self, frame, x, y):
        print('start sgbm')
        # 左镜头的内参，如焦距
        left_camera_matrix = np.array([[516.5066236,-1.444673028,320.2950423],[0,516.5816117,270.7881873],[0.,0.,1.]])
        right_camera_matrix = np.array([[511.8428182,1.295112628,317.310253],[0,513.0748795,269.5885026],[0.,0.,1.]])

        # 畸变系数,K1、K2、K3为径向畸变,P1、P2为切向畸变
        left_distortion = np.array([[-0.046645194,0.077595167, 0.012476819,-0.000711358,0]])
        right_distortion = np.array([[-0.061588946,0.122384376,0.011081232,-0.000750439,0]])

        # 旋转矩阵
        R = np.array([[0.999911333,-0.004351508,0.012585312],
                    [0.004184066,0.999902792,0.013300386],
                    [-0.012641965,-0.013246549,0.999832341]])
        # 平移矩阵
        T = np.array([-120.3559901,-0.188953775,-0.662073075])

        size = (640, 480)

        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                        right_camera_matrix, right_distortion, size, R,
                                                                        T)
        
        # 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
        left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
        right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)
        # 切割为左右两张图片
        frame1 = frame[0:480, 0:640]
        frame2 = frame[0:480, 640:1280]
        # 将BGR格式转换成灰度图片，用于畸变矫正
        imgL = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        imgR = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # 重映射，就是把一幅图像中某位置的像素放置到另一个图片指定位置的过程。
        # 依据MATLAB测量数据重建无畸变图片,输入图片要求为灰度图
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
        img_channels = 3
        stereo = cv2.StereoSGBM_create(minDisparity=1,
                                    numDisparities=64,
                                    blockSize=blockSize,
                                    P1=8 * img_channels * blockSize * blockSize,
                                    P2=32 * img_channels * blockSize * blockSize,
                                    disp12MaxDiff=-1,
                                    preFilterCap=1,
                                    uniquenessRatio=10,
                                    speckleWindowSize=100,
                                    speckleRange=100,
                                    mode=cv2.STEREO_SGBM_MODE_HH)
        # 计算视差
        disparity = stereo.compute(img1_rectified, img2_rectified)
        
        threeD = cv2.reprojectImageTo3D(disparity, Q, handleMissingValues=True)
        # 计算出的threeD，需要乘以16，才等于现实中的距离
        threeD = threeD * 16
        # return point_x, point_y, point_z
        point = Point()
        point.x, point.y, point.z = threeD[y][x][0] / 1000.0, threeD[y][x][1] / 1000.0, threeD[y][x][2] / 1000.0    
        
        return point

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = CameraPublisher()
    node.run()
