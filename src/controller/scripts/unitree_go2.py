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
import actionlib
from interfaces.msg import Go2Action, Go2Goal, Go2Feedback, Go2Result
# form 

class UnitreeGo2:
    def __init__(self) -> None:
        rospy.init_node('unitree_go2', anonymous=True)
        rospy.loginfo("节点:unitree_go2, 已启动!")
        # rospy.Subscriber('Unitree_Highcmd', HighCmd, self.callback)

        #add action serve
        self.server = actionlib.SimpleActionServer('go2_serve',Go2Action,self.go2_callback,False)
        self.server.start()
        rospy.loginfo("启动服务端")

        # from unitree_sdk_python init a sport client
        self.client = SportClient() 
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.vx = 0
        self.vy = 0
        self.vyaw = 0

        # Create a thread for continuous movement
        self.move_thread = threading.Thread(target=self.unitree_move_thread)
        self.move_thread.start()
        
        # Robot state
        self.robot_state = []
        self.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)

    def HighStateHandler(self, msg: SportModeState_):
        self.robot_state = msg.position

    def go2_callback(self,goal):
        self.sub.Init(self.HighStateHandler, 1)
        time.sleep(0.01)
        target_position_x = goal.target_position[0] + self.robot_state[0]    # 目标地点，格式为[x, y] 
        target_position_y = goal.target_position[1] + self.robot_state[1]
        # print(f'x= {target_position_x}')
        
        feedback = Go2Feedback()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.sub.Init(self.HighStateHandler, 1)
            time.sleep(0.01)
            
            current_position = self.robot_state[:2]
            
            feedback.current_position = current_position
            self.server.publish_feedback(feedback)
            
            if goal.target_position[0] != 0:
                self.vx = 0.1
            if goal.target_position[1] != 0:
                self.vy = 0.1
            
            print(f'current_data = {current_position}')
            print(f'target_position_x = {target_position_x}, target_position_y = {target_position_y}')
            print(f'x绝对值 = {abs(current_position[0] - target_position_x)}')
            print(f'y绝对值 = {abs(current_position[1] - target_position_y)}')
            # 判断是否达到目标地点 
            if (abs(current_position[0] - target_position_x) < 0.05) and (abs(current_position[1] - target_position_y) < 0.05):
                result = Go2Result()
                result.final_position = current_position
                self.server.set_succeeded(result)
                break
            else:
                if abs(current_position[0] - target_position_x) < 0.05:
                    self.vx = 0.0
                
                if abs(current_position[1] - target_position_y) < 0.05:
                    self.vy = 0.0

            rate.sleep()



    def unitree_move_thread(self):
        rate = rospy.Rate(10)  # 10 Hz, adjust as needed
        while not rospy.is_shutdown():        
            if(self.vx + self.vy + self.vyaw) != 0.0 :   #速度不为0，并且停止所有运动
                self.client.Move(self.vx, self.vy, self.vyaw)
                rate.sleep()
            # else:
            #     rospy.loginfo("flag = %d", self.mode_change_flag)
            #     if self.mode == 1 and self.client.StopMove() == 0 and self.mode_change_flag == 1:
            #         while self.client.StandUp() != 0:
            #             time.sleep(1)
            #         self.mode_change_flag = 0

            #     if self.mode == 0 and self.client.StopMove() == 0 and self.mode_change_flag == 1: 
            #         while self.client.StandDown() != 0:
            #             time.sleep(1)
            #         self.mode_change_flag = 0

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ChannelFactoryInitialize(0, 'eno1')  # 改称网口名字

    node = UnitreeGo2()
    node.run()
