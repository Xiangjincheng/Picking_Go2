import actionlib.simple_action_client
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces.msg import Rois, Go2Target
from interfaces.srv import *
#add action
import actionlib
from interfaces.msg import Go2Action, Go2Goal, Go2Feedback, Go2Result

class Manage:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")
        self.bridge=CvBridge()

        rospy.Subscriber('rois', Rois, self.rois_callback)     
        rospy.Subscriber('go2_target', Go2Target, self.go2_target_callback)
        self.roi_to_point_client = rospy.ServiceProxy('roi_to_point_serve', RoiToPoint) # type: ignore
        # self.arm_client = rospy.ServiceProxy('arm_serve', ArmCtrl)
        
        #add action client
        self.go2_client = actionlib.SimpleActionClient('go2_serve', Go2Action)
        self.go2_client.wait_for_server()
        
    def rois_callback(self, rois_msg):
        for roi in rois_msg.rois:
            rospy.wait_for_service('roi_to_point_serve')
            targets = self.roi_to_point_client(roi)
            print(targets)
            #加入Go2运动
            if():




                pass
            #Go2运动完毕
            # if(targets_z != 0):
            #     rospy.wait_for_service('arm_controller')
            #     flag = self.arm_client(targets)
            #     if(flag):
            #         pass
    
    def go2_target_callback(self, msg):
        self.goal = Go2Goal()
        self.goal.target_position = msg.target_position
        self.go2_client.send_goal(self.goal, self.result_callback, self.recive_callback, self.feedback_callback)

    def recive_callback(self):
        rospy.loginfo("目标已被处理...") 

    def result_callback(self, state, result):
        rospy.loginfo("最终位置: x = %f, y = %f" % (result.final_position[0], result.final_position[1]))
        
    def feedback_callback(self, feedback):
        rospy.loginfo("当前执行位置: x = %f, y = %f" % (feedback.current_position[0], feedback.current_position[1]))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Manage()
    node.run()
