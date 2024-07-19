import actionlib.simple_action_client
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from interfaces.msg import Rois
from interfaces.srv import *
import threading
#add action
import actionlib
from interfaces.msg import Go2Action, Go2Goal, Go2Feedback, Go2Result

class RoisToPoint:
    def __init__(self):
        rospy.init_node('rois_to_point', anonymous=True)
        rospy.loginfo("节点:rois_to_point, 已启动!")
        self.bridge=CvBridge()

        rospy.Subscriber('rois', Rois, self.rois_callback)     
        self.roi_to_point_client = rospy.ServiceProxy('roi_to_point_serve', RoiToPoint) # type: ignore
        # self.arm_client = rospy.ServiceProxy('arm_serve', ArmCtrl)
        
        #add action client
        self.go2_client = actionlib.SimpleActionClient('go2_serve', Go2Action)
        rospy.loginfo("等待Action Server启动...")
        self.go2_client.wait_for_server()
        rospy.loginfo("Action Server已启动")
        
        self.goal = Go2Goal()
        self.goal.target_position = [0.3,0]
        self.go2_client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)

        #go2的运动控制
        self.vx = 0
        self.vy = 0
        self.vyaw = 0

        # Create a thread for continuous movement
        self.move_thread = threading.Thread(target=self.unitree_move_thread)
        self.move_thread.start()

    def done_cb(self, state, result):
        rospy.loginfo("最终位置: x = %f, y = %f" % (result.final_position[0], result.final_position[1]))
        
    def active_cb(self):
        rospy.loginfo("目标已被处理...") 
        
    def feedback_cb(self, feedback):
        rospy.loginfo("当前执行位置: x = %f, y = %f" % (feedback.current_position[0], feedback.current_position[1]))
        
    def rois_callback(self, rois_msg):
        for roi in rois_msg.rois:
            rospy.wait_for_service('roi_to_point_serve')
            targets = Point()
            targets = self.roi_to_point_client(roi)
            targets_x = targets[0],targets_y = targets[1],targets_z = targets[2]
            print(targets_x, targets_y, targets_z)
            #加入Go2运动
            if():




                pass
            #Go2运动完毕
            # if(targets_z != 0):
            #     rospy.wait_for_service('arm_controller')
            #     flag = self.arm_client(targets)
            #     if(flag):
            #         pass

    def unitree_move_thread(self):
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():        
            if(self.vx + self.vy + self.vyaw) != 0.0 :   #速度不为0，并且停止所有运动
                self.client.Move(self.vx, self.vy, self.vyaw)
                rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = RoisToPoint()
    node.run()
