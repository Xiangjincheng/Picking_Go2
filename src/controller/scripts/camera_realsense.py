import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSense:
    def __init__(self):
        rospy.init_node('realsense', anonymous=True)
        rospy.loginfo("节点:realsense, 已启动!")

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)

        self.bridge = CvBridge()

        self.publisher_image = rospy.Publisher('rs_image', Image, queue_size = 10)
        self.timer1 = rospy.Timer(rospy.Duration(1/10.0), self.timer1_callback)

        self.publisher_mis_distance = rospy.Publisher('rs_distance', Float64, queue_size = 10)
        self.timer2 = rospy.Timer(rospy.Duration(1/10.0), self.timer2_callback)

        self.publisher_align_image = rospy.Publisher('align_image', Image, queue_size = 10)
        self.timer3 = rospy.Timer(rospy.Duration(1/10.0), self.timer3_callback)

    def timer1_callback(self, event):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())
        image = np.hstack((color_image, depth_colormap))
        msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        #msg = self.bridge.cv2_to_imgmsg(image 'bgr8')
        self.publisher_image.publish(msg)

    def timer2_callback(self, event):
        msg = Float64()
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return
        depth_image = np.asanyarray(depth_frame.get_data())
        msg.data = depth_image.mean()/1000
        self.publisher_mis_distance.publish(msg)   # mean distance 
    
    def timer3_callback(self, event):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
        depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
        color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        color_image = np.asanyarray(color_frame.get_data())
        image = np.hstack((color_image, depth_colormap))
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        #msg = self.bridge.cv2_to_imgmsg(image 'bgr8')

        self.publisher_align_image.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RealSense()
    node.run()
