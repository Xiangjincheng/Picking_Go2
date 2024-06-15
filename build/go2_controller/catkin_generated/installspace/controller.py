
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

class ControllerNode:
    def __init__(self) -> None:
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo('Controller node started')
        rospy.Subscriber('Unitree_Highcmd', HighCmd, self.callback)

        #from unitree_sdk_python init a sport client
        # self.client = SportClient() 
        # self.client.SetTimeout(10.0)
        # self.client.Init()

    def callback(self, highcmd):
        
        # self.client.Move(0.3, 0, 0.3)
        #vx:  取值范围[-2.5~5] (m/s)； vy:  取值范围[-2.5~5] (m/s)； vyaw:  取值范围[-4~4] (rad/s)

        rospy.loginfo(
            "V_x = %f, V_y = %f, Yaw_speed = %f", 
            highcmd.velocity[0], highcmd.velocity[1], highcmd.mode
        )

    def run(self):
        # spin() simply keeps Python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    # if len(sys.argv)>1:
    #     ChannelFactoryInitialize(0, sys.argv[1])
    # else:
    #     ChannelFactoryInitialize(0)
    # time.sleep(1)

    controller_node = ControllerNode()
    controller_node.run()
