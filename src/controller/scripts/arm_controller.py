import rospy
import sys
import os
import time
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
kinematics_dir = os.path.join(current_dir, "..", "armfpv_sdk")
sys.path.append(kinematics_dir)
from ArmIK.ArmMoveIK import *
from HiwonderSDK.Board import setBusServoPulse

from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_msgs.msg import RawIdPosDur
from geometry_msgs.msg import Point

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        rospy.loginfo("节点:arm_controller, 已启动!")

        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # rospy.Subscriber("joy", Joy, self.joy_callback)
        # self.joy_pub = rospy.Publisher('joint_state', HighCmd, queue_size=10)
        self.AK = ArmIK()
        self.AK.setPitchRangeMoving((18, 0, 25), 0, -180, 180, 1000)

    def camera_trans_base(self, target_camera):
        transformation_matrix = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, 8],   #add error
            [0, 0, 1, 27],
            [0, 0, 0, 1]
        ])
        target_camera_homogeneous = np.append(target_camera, 1)
        target_base_homogeneous = np.dot(transformation_matrix, target_camera_homogeneous)
        target_base = target_base_homogeneous[:3] / target_base_homogeneous[3]
        #exchange x,y
        return target_base[1], target_base[0], target_base[2]


    def obj_pos_callback(self, obj_pos_msgs):
        x_dis, y_dis, z_dis = self.camera_trans_base(np.array([obj_pos_msgs.x, obj_pos_msgs.y, obj_pos_msgs.z]))

        rospy.loginfo(f"接收到的目标物体坐标: x={x_dis}, y={y_dis}, z={z_dis}")
        
        self.AK.setPitchRangeMoving((x_dis, y_dis, z_dis), -90, -180, 180, 1000)
        time.sleep(1)
        
        self.grab_object()

    def grab_object(self):
 
        rospy.loginfo("开始抓取目标物体")
        setBusServoPulse(1, 500, 500)  #pitch object
        time.sleep(1)
        rospy.loginfo("抓取动作完成")


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = ArmController()
    node.run()



