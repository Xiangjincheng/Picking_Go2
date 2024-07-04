#!/usr/bin/env python3
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_msgs.msg import RawIdPosDur


def set_servos(pub, duration, pos_s):
    msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(x[0], x[1], duration), pos_s)))
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('test_msg', anonymous=True)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    while True:
        try:
            set_servos(joints_pub, 1000, ((6, 300), (1, 200)))  #直接控制舵机角度 0- 1000
            rospy.sleep(1)
            set_servos(joints_pub, 1000, ((6, 650), (1, 500)))
            rospy.sleep(1)
        except KeyboardInterrupt:
            break
