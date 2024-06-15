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

        # Initialize threading event for controlling thread execution
        self.event = threading.Event()
        self.move_thread = threading.Thread(target=self.move_thread_func)
        self.move_thread.start()

        self.move_data = [0, 0, 0]

    def callback(self, highcmd):
        if highcmd.mode == 1:
            self.client.StandUp()
        else:
            self.client.StandDown() 
            self.move_data = [highcmd.velocity[0], highcmd.velocity[1], highcmd.yawSpeed]

    def move_thread_func(self):
        while not rospy.is_shutdown():
            self.event.wait()

            self.client.Move(self.move_data[0], self.move_data[0], self.move_data[0])
            rospy.loginfo(self.move_data)

            self.event.clear()

    def run(self):
        # spin() simply keeps Python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    ChannelFactoryInitialize(0, 'enp2s0') #改称网口名字
    time.sleep(1)

    controller_node = ControllerNode()
    try:
        controller_node.run()
    except rospy.ROSInterruptException:
        pass
