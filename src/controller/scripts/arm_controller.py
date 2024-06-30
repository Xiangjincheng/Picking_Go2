import rospy
import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
arm_bus_servo = os.path.join(current_dir, "..", "arm_bus_servo")
sys.path.append(arm_bus_servo)
import Board


class ArmController:
    def __init__(self):
        rospy.init_node('arm_controller', anonymous=True)
        rospy.loginfo("节点:arm_controller, 已启动!")
        
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.joy_pub = rospy.Publisher('joint_state', HighCmd, queue_size=10)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    node = ArmController()
    node.run()
