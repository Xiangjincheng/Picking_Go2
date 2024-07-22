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
from interfaces.srv import ArmCtrl, ArmCtrlResponse

class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        rospy.loginfo("节点:arm_controller, 已启动!")

        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.arm_serve = rospy.Service('arm_serve', ArmCtrl, self.arm_serve_callback)
        #rospy.Subscriber("obj_pos", Point, self.obj_pos_callback)
        
        self.AK = ArmIK()
        self.arm_init()
        self.processing = False  # 初始化处理状态标志

    def arm_serve_callback(self, request):
        obj_pos = request.targets
        responce = ArmCtrlResponse()

        x_dis, y_dis, z_dis = self.camera_trans_base(np.array([obj_pos.x, obj_pos.y, obj_pos.z]))
        rospy.loginfo(f"接收到的目标物体坐标: x={x_dis}, y={y_dis}, z={z_dis}")
        move_callback = self.AK.setPitchRangeMoving((x_dis, y_dis, z_dis), 0, -180, 180, 1000)
        print(move_callback)
        time.sleep(3)
        self.grab_object()
        time.sleep(1)
        self.AK.setPitchRangeMoving((-15,0,15), -135, -180, 180, 1000)
        time.sleep(1)
        self.AK.setPitchRangeMoving((15,0,15), 135, 0, 180, 1000)
        time.sleep(1)
        setBusServoPulse(1, 0, 500)
        time.sleep(1)
        self.AK.setPitchRangeMoving((-15,0,15), -135, -180, 0, 1000)
        time.sleep(1)
        self.arm_init()
        if move_callback != False:
            move_callback = True
            
        responce.success = move_callback
        return responce
    
        

    def camera_trans_base(self, target_camera):
        print(target_camera)
        transformation_matrix = np.array([
            [0, 0, -1, 13], 
            [1, 0, 0, -2],   # 添加误差
            [0, -1, 0, 26],
            [0, 0, 0, 1]
        ])
        target_camera_homogeneous = np.append(target_camera, 1)
        target_base_homogeneous = np.dot(transformation_matrix, target_camera_homogeneous)
        target_base = target_base_homogeneous[:3] / target_base_homogeneous[3]
        return target_base[0], target_base[1], target_base[2]

    #def obj_pos_callback(self, obj_pos_msgs):
     #   if not self.processing:  # 检查是否有正在进行的处理
      #      self.processing = True
      #      x_dis, y_dis, z_dis = self.camera_trans_base(np.array([obj_pos_msgs.x, obj_pos_msgs.y, obj_pos_msgs.z]))
#
       #     rospy.loginfo(f"接收到的目标物体坐标: x={x_dis}, y={y_dis}, z={z_dis}")
       #     
       #     self.AK.setPitchRangeMoving((x_dis, y_dis, z_dis), 0, -180, 180, 1000)
       #     rospy.Timer(rospy.Duration(1), self.complete_action, oneshot=True) 

    def complete_action(self, event):
        self.grab_object()  
        self.processing = False  # 重置处理状态标志

    def arm_init(self):
        self.AK.setPitchRangeMoving((-3, 0, 18), 0, -180, 180, 1000)
        time.sleep(1)
        setBusServoPulse(1, 0, 500)  # 初始抓取位置
        time.sleep(1)

    def grab_object(self):
        rospy.loginfo("开始抓取目标物体")
        setBusServoPulse(1, 500, 500)  # 抓取动作
        time.sleep(1)
        rospy.loginfo("抓取动作完成")
        # self.arm_init()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ArmController()
    node.run()

# scp unitree_noetic/src/controller/scripts/arm_controller.py cheng@192.168.123.222:~/unitree_noetic/src/controller/scripts/
