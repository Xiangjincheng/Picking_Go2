import actionlib.simple_action_client
import rospy
import numpy as np
import time
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import  RegionOfInterest
from geometry_msgs.msg import Point
from interfaces.srv import *
#add action
import actionlib
from interfaces.msg import Go2Action, Go2Goal, Go2Feedback, Go2Result
import math

class Manage:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")

        self.position_offset = []
        self.target_history = []
        self.error_threshold = 0.2
        self.history_length = 4
        self.bridge=CvBridge()

        rospy.Subscriber('region_of_interest', RegionOfInterest, self.rois_callback)     
        self.roi_to_point_client = rospy.ServiceProxy('roi_to_point_serve', RoiToPoint) 
        self.arm_client = rospy.ServiceProxy('arm_serve', ArmCtrl)
        
        # add action client
        self.go2_client = actionlib.SimpleActionClient('go2_serve', Go2Action)
        self.go2_client.wait_for_server()

        self.go2_move([0, -0.15])

    def rois_callback(self, roi_msg):
        if roi_msg.x_offset != 0:
            rospy.wait_for_service('roi_to_point_serve')
            target = self.roi_to_point_client(roi_msg).target
            if self.check_target(target):
                stable_target = self.check_target_is_stable(target)

                if stable_target.x != 0:
                    print(f'稳定数据{stable_target}')
                    self.position_offset = []
                    self.go2_move([-0.15, 0])
                    if self.position_offset != []:
                        stable_target.x = stable_target.x + self.position_offset[1]*100
                        stable_target.z = stable_target.z + self.position_offset[0]*100
                        rospy.wait_for_service('arm_serve')
                        response = self.arm_client(stable_target)         
                        print(response)
        else:
            self.go2_move([0.35, 0])

    def check_target_is_stable(self, target):
        stable_result = Point()
        if self.target_history == []:
            self.target_history.append(target)
        elif len(self.target_history) == self.history_length:
            avg_x = sum(t.x for t in self.target_history) / self.history_length
            avg_y = sum(t.y for t in self.target_history) / self.history_length
            avg_z = sum(t.z for t in self.target_history) / self.history_length
            stable_result = Point(avg_x, avg_y, avg_z)
            self.target_history = []
        else:
            if abs(self.target_history[-1].x - target.x) < self.error_threshold and \
               abs(self.target_history[-1].y - target.y) < self.error_threshold and \
               abs(self.target_history[-1].z - target.z) < self.error_threshold:
                self.target_history.append(target)
            else:
                self.target_history = []

        return stable_result
    
    def check_target(self, target):
        check_result = True

        if target is None:
            check_result = False

        if math.isinf(target.x) or math.isnan(target.x) or \
        math.isinf(target.y) or math.isnan(target.y) or \
        math.isinf(target.z) or math.isnan(target.z):
            check_result = False

        if target.z > 100.0:
            check_result = False

        return check_result
    
    def go2_move(self, move_position):
        self.goal = Go2Goal()
        target_position = move_position
        self.goal.target_position = target_position
        self.go2_client.send_goal(self.goal, self.result_callback, self.active_callback, self.feedback_callback)

    def active_callback(self):
        rospy.loginfo("目标已被处理...") 

    # need compare result with goal
    def result_callback(self, state, result):
        rospy.loginfo("初始位置: x = %f, y = %f" % (result.start_position[0], result.start_position[1]))
        rospy.loginfo("最终位置: x = %f, y = %f" % (result.final_position[0], result.final_position[1]))
        self.position_offset.append(result.final_position[0]-result.start_position[0])
        self.position_offset.append(result.final_position[1]-result.start_position[1])
            
    def feedback_callback(self, feedback):
        rospy.loginfo("当前执行位置: x = %f, y = %f" % (feedback.current_position[0], feedback.current_position[1]))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Manage()
    node.run()
