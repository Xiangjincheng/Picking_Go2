import rospy
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
kinematics_dir = os.path.join(current_dir, "..", "Kinematics")
sys.path.append(kinematics_dir)
from ik_transform import ArmIK

from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_msgs.msg import RawIdPosDur


class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        rospy.loginfo("节点:arm_controller, 已启动!")

        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        # rospy.Subscriber("joy", Joy, self.joy_callback)
        # self.joy_pub = rospy.Publisher('joint_state', HighCmd, queue_size=10)
        AK = ArmIK()
        target = AK.setPitchRanges((0, 0.26, 0.04), -180, -180, 0)
        if target:
            servo_data = target[1]
            self.set_servos(self.joints_pub, 1000, ((1, 450), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))


    def set_servos(pub, duration, pos_s):
        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(x[0], x[1], duration), pos_s)))
        pub.publish(msg)

    def run(self):
        rospy.spin()

# 舵机发布

rospy.sleep(0.2)


if __name__ == '__main__':
    
    node = ArmController()
    node.run()



