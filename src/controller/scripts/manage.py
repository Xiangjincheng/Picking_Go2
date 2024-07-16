import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces.msg import Rois
from interfaces.srv import *

class RoisToPoint:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")
        self.bridge=CvBridge()

        rospy.Subscriber('rois', Rois, self.rois_callback)     

        self.roi_to_point_client = rospy.ServiceProxy('roi_to_point_serve', RoiToPoint)
        self.arm_client = rospy.ServiceProxy('arm_serve', ArmCtrl)

    def rois_callback(self, rois_msg):
        for roi in rois_msg.rois:
            rospy.wait_for_service('depth_serve')
            target = self.roi_to_point_client(roi)
            print(target)
            # if(target):            
            #     rospy.wait_for_service('arm_controller')
            #     flag = self.arm_client(target)
            #     if(flag):
            #         pass


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RoisToPoint()
    node.run()
