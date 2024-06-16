# 手柄控制
import rospy
from sensor_msgs.msg import Joy
from unitree_legged_msgs.msg import HighCmd

class JoystickController:
    def __init__(self):
        rospy.init_node('joystick_controller', anonymous=True)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.joy_pub = rospy.Publisher('Unitree_Highcmd', HighCmd, queue_size=10)
        self.max_linear_vel = 0.5  # 最大线速度
        self.max_angular_vel = 1.0  # 最大角速度
        self.highcmd_mode = 1

    def joy_callback(self, joy_data):
        highcmd = HighCmd()
        highcmd.velocity[0] = self.max_linear_vel * joy_data.axes[0]    
        highcmd.velocity[1] = self.max_linear_vel * joy_data.axes[1]    

        if(joy_data.buttons[0] == 1): #A
            self.highcmd_mode = 1    #站起

        if(joy_data.buttons[1] == 1): #B
            self.highcmd_mode = 0    
        
        highcmd.mode = self.highcmd_mode 

        self.joy_pub.publish(highcmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = JoystickController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
