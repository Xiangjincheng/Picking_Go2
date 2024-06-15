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

        # Create a thread for continuous movement
        self.move_thread = threading.Thread(target=self.move_continuous)
        self.move_thread.start()

        self.vel = [0, 0]
        self.vyaw = 0

    def callback(self, highcmd):
        if highcmd.mode == 1:
            self.client.StandUp()
        else:
            self.client.StandDown()
        
        rospy.loginfo(
            "V_x = %f, V_y = %f, vyaw = %f", 
            highcmd.velocity[0], highcmd.velocity[1], highcmd.yawSpeed
        )
        self.vel = [highcmd.velocity[0], highcmd.velocity[1]]
        self.vyaw = highcmd.yawSpeed

    def move_continuous(self):
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():
            # Example of continuous movement with a fixed speed
            self.client.Move(self.vel[0], self.vel[1], self.vyaw)
            rate.sleep()

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
