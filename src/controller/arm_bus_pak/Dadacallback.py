#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np

# 初始化发布器
pub = rospy.Publisher('/data', JointState, queue_size=100)

def data_callback(joint):
    data = np.zeros(6)  # Initialize pwm as a NumPy array of zeros

    data[0] = joint.position[0]*477.464837+500
    data[1] = joint.position[1]*477.464837+500
    data[2] = joint.position[2]*477.464837+500
    data[3] = joint.position[3]*477.464837+500
    data[4] = joint.position[4]*477.464837+500

    for i in range(5):
        rospy.loginfo("joint%d: data is %d!", i + 1, data[i])

    joint.effort = data  
    
    pub.publish(joint)

def main():
    # 初始化节点
    rospy.init_node('get_data', anonymous=True)
    # 订阅joint_states主题
    rospy.Subscriber('/joint_states', JointState, data_callback)
    # 保持节点运行，等待回调函数处理消息
    rospy.spin()

if __name__ == '__main__':
    main()

