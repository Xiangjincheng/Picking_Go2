import actionlib.simple_action_client
import rospy
import numpy as np
import time
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import  RegionOfInterest, Image, CameraInfo
from geometry_msgs.msg import Point
from interfaces.srv import *
#add action
import actionlib
from interfaces.msg import Go2Action, Go2Goal, Go2Feedback, Go2Result, Rois
import pyrealsense2 as rs 
import math

class Manage:
    def __init__(self):
        rospy.init_node('manage_node', anonymous=True)
        rospy.loginfo("节点:manage_node, 已启动!")

        self.stable_target = Point()
        self.depth = Image()
        self.intrinsics = rs.intrinsics()
        self.target_history = []
        self.error_threshold = 0.5
        self.history_length = 2
        self.bridge=CvBridge()

        rospy.Subscriber('region_of_interest', Rois, self.rois_callback)     
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.rs_depth_callback)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.intrinsics_callback)

        self.arm_client = rospy.ServiceProxy('arm_serve', ArmCtrl)
        
        # add action client
        self.go2_client = actionlib.SimpleActionClient('go2_serve', Go2Action)
        self.go2_client.wait_for_server()

    def rois_callback(self, roi_msg):
        if roi_msg.rois[0].x_offset != 0:
            for e in roi_msg.rois:
                mid_x=e.x_offset +e.width*0.5
                mid_y=e.y_offset +e.height *0.5
                mid_x=int(mid_x)
                mid_y=int(mid_y)
                depth_image = self.bridge.imgmsg_to_cv2(self.depth)
                target = Point()
                if depth_image[mid_y][mid_x] != 0:
                    camera_coordinate = rs.rs2_deproject_pixel_to_point(self.intrinsics, [mid_x,mid_y], depth_image[mid_y][mid_x]/1000)
                    target.x, target.y, target.z = camera_coordinate[0], camera_coordinate[1], camera_coordinate[2]
                if self.check_target(target):
                    rospy.wait_for_service('arm_serve')
                    print(f"new stable_target = {self.stable_target}")
                    response = self.arm_client(self.stable_target)     


            '''
            rospy.wait_for_service('roi_to_point_serve')
            target = self.roi_to_point_client(roi_msg).target
            print(f'target = {target}')
            if self.check_target(target):
                stable_target = self.check_target_is_stable(target)
                if stable_target.x != 0:
                    self.stable_target = stable_target
                    self.go2_move([0, 0.35])   
            '''


       # else:
       #     self.go2_move([0.35, 0])

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
        print(f"old stable_target = {self.stable_target}")
        time.sleep(1)
        x_offset = result.final_position[0]-result.start_position[0]
        y_offset = result.final_position[1]-result.start_position[1]
        self.stable_target.x = self.stable_target.x - x_offset*100
        self.stable_target.z = self.stable_target.z - y_offset*100
        rospy.wait_for_service('arm_serve')
        print(f"new stable_target = {self.stable_target}")
        response = self.arm_client(self.stable_target)         
        print(response)
            
    def feedback_callback(self, feedback):
        pass
        rospy.loginfo("当前执行位置: x = %f, y = %f" % (feedback.current_position[0], feedback.current_position[1]))

    def intrinsics_callback(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.k[2]
        self.intrinsics.ppy = msg.k[5]
        self.intrinsics.fx = msg.k[0]
        self.intrinsics.fy = msg.k[4]
        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in msg.d]

    def rs_depth_callback(self, msg):
        self.depth = msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Manage()
    node.run()
