import rospy
from unitree_legged_msgs.msg import HighCmd

import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)

import threading

class ControllerNode:
    def __init__(self) -> None:
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo('Controller node started')
        rospy.Subscriber('Unitree_Highcmd', HighCmd, self.callback)

        # from unitree_sdk_python init a sport client
        self.client = SportClient() 
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.mode = 1   #standup mode
        self.vx = 0
        self.vy = 0
        self.vyaw = 0

        # Create a thread for continuous movement
        self.move_thread = threading.Thread(target=self.unitree_move_thread)
        self.move_thread.start()

    def callback(self, highcmd):
        self.mode = highcmd.mode
        self.vx = highcmd.velocity[0]
        self.vy = highcmd.velocity[1]
        self.vyaw = highcmd.yawSpeed

    def unitree_move_thread(self):
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():            
            if(self.vx + self.vy + self.vyaw) != 0.0 and self.client.StopMove() == 0:   #速度不为0，并且停止所有运动
                self.client.Move(self.vx, self.vy, self.vyaw)
                rate.sleep()
            if self.mode == 1 and self.client.StopMove() == 0:
                self.client.StandUp()

            if self.mode == 0 and self.client.StopMove() == 0:
                self.client.StandDown()

    def run(self):
        # spin() simply keeps Python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ChannelFactoryInitialize(0, 'eno1')  # 改称网口名字
    time.sleep(1)

    controller_node = ControllerNode()
    try:
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
