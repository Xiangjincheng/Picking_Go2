import actionlib.simple_action_client
import rospy
import numpy as np
import time
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import  RegionOfInterest, Image, CameraInfo
from unitree_legged_msgs.msg import HighCmd
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
        self.go2_pub = rospy.Publisher('Unitree_Highcmd', HighCmd, queue_size=10)
        # add action client
        self.go2_client = actionlib.SimpleActionClient('go2_serve', Go2Action)
        self.go2_client.wait_for_server()

    def rois_callback(self, roi_msg):
        if len(roi_msg.rois) != 0:
            self.go2_move_by_velocity(0.0, 0.0) 
            for e in roi_msg.rois:
                mid_x=e.x_offset +e.width*0.5
                mid_y=e.y_offset +e.height *0.5
                mid_x=int(mid_x)
                mid_y=int(mid_y)
                depth_image = self.bridge.imgmsg_to_cv2(self.depth)
                target = Point()
                if depth_image[mid_y][mid_x] != 0:
                    camera_coordinate = rs.rs2_deproject_pixel_to_point(self.intrinsics, [mid_x,mid_y], depth_image[mid_y][mid_x]/10)
                    target.x, target.y, target.z = camera_coordinate[0], camera_coordinate[1], camera_coordinate[2]
		
                print(f'depth = {target.z}')
                if self.check_target(target):
                    if target.z > 35:
                        self.go2_move_by_line([0.0, (target.z - 35) / 100.0])
                        break
                    if abs(target.x) > 20:
                        self.go2_move_by_line([target.x / 100.0, 0.0])
                        break                    
                    rospy.wait_for_service('arm_serve')
                    print(f"target = {target}")
                    response = self.arm_client(target)   
                    print(f'response = {response}') 

        else:
            self.go2_move_by_velocity(0.1, 0.0)   #vx, vy
            
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
            
        if target.z == 0:
            check_result = False

        return check_result
    
    def go2_move_by_line(self, move_position):
        rospy.loginfo(f"target_position = {move_position}") 
        self.goal = Go2Goal()
        target_position = move_position
        self.goal.target_position = target_position
        self.go2_client.send_goal(self.goal, self.result_callback, self.active_callback, self.feedback_callback)

    def go2_move_by_velocity(self, v_x, v_y):
        highcmd = HighCmd()
        highcmd.velocity[0] = v_x
        highcmd.velocity[1] = v_y
        self.go2_pub.publish(highcmd)
        rospy.loginfo(f"go2_pub.publish...{v_x, v_y}") 

    def active_callback(self):
        rospy.loginfo("目标已被处理...") 

    # need compare result with goal
    def result_callback(self, state, result):
        rospy.loginfo("初始位置: x = %f, y = %f" % (result.start_position[0], result.start_position[1]))
        rospy.loginfo("最终位置: x = %f, y = %f" % (result.final_position[0], result.final_position[1]))
            
    def feedback_callback(self, feedback):
        pass
        rospy.loginfo("当前执行位置: x = %f, y = %f" % (feedback.current_position[0], feedback.current_position[1]))

    def intrinsics_callback(self, msg):
        self.intrinsics.width = msg.width
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = msg.K[2]
        self.intrinsics.ppy = msg.K[5]
        self.intrinsics.fx = msg.K[0]
        self.intrinsics.fy = msg.K[4]
        if msg.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif msg.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in msg.D]

    def rs_depth_callback(self, msg):
        self.depth = msg

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = Manage()
    node.run()
