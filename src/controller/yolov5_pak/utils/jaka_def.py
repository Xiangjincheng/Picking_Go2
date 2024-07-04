import jkrc
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import numpy as np
import math



def reset(): # 复位
    ls = [90,90,-90,-180,90,-180] #first

    #ls=[100.901,84.278,-58.513,-192.671,115.369,-185.872]
    ret_joint_pos=(PI * np.array(ls) / 180.0).tolist()
    robot.joint_move(joint_pos=ret_joint_pos, move_mode=ABS, is_block=False, speed=5)  # 绝对运动
    return

def move_blue():
    ls = [-61.598, 64.059, -87.419, -32.756, 110.271, -190.337]
    ret_joint_pos = (PI * np.array(ls) / 180.0).tolist()
    robot.joint_move(joint_pos=ret_joint_pos, move_mode=ABS, is_block=False, speed=5)  # 绝对运动
    tcp_pos = [10, 0, 0, 0, 0, 0]
    robot.linear_move(tcp_pos, INCR, True, 20)
    return

def line_mv(x,y,z,speed=10): # 直线移动35
    #tcp_pos = [-55, 520, 110, 0, 0, 0]
    tcp_pos=[x,y,z, 0, 0, 0]
    robot.linear_move(tcp_pos, INCR, True, speed)
    return

def nijie(x,y,z):
    '''
        cartesian_pose:目标末端位姿  list   [x,y,z,rx,ry,rz]  rx,ry,rz弧度制
        [9.232,415.073,505.980,-110.786*PI/180,89.572*PI/180,-21.316*PI/180]
    '''
    ret=robot.get_joint_position() # 获取当前关节角度
    ref_pos=ret[1]

    ret=robot.get_tcp_position() # 当前位姿
    now_tcp_pos=ret[1]

    cartesian_pose=[x,y,z,now_tcp_pos[3],now_tcp_pos[4],now_tcp_pos[5]]
    print(cartesian_pose)
    ret = robot.kine_inverse(ref_pos,cartesian_pose) # 求目标末端位姿 的 关节角度
    print(ret)
    joint_pos=list(ret[1])
    robot.joint_move(joint_pos=joint_pos,
                     move_mode=0,
                     is_block=True,
                     speed=0.8)
    return