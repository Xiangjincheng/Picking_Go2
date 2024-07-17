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
import math


class SportModeTest:
    def __init__(self) -> None:
        # Time count
        self.t = 0
        self.dt = 0.01

        # Initial poition and yaw
        self.px0 = 0
        self.py0 = 0
        self.yaw0 = 0

        self.client = SportClient()  # Create a sport client
        self.client.SetTimeout(10.0)
        self.client.Init()

    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    def StandUpDown(self):
        self.client.StandDown()
        print("Stand down !!!")
        time.sleep(1)

        self.client.StandUp()
        print("Stand up !!!")
        time.sleep(1)

        self.client.StandDown()
        print("Stand down !!!")
        time.sleep(1)

        self.client.Damp()

    def VelocityMove(self):
        elapsed_time = 1
        for i in range(int(elapsed_time / self.dt)):
            self.client.Move(0.3, 0, 0.3)  # vx, vy vyaw
            time.sleep(self.dt)
        self.client.StopMove()

    def BalanceAttitude(self):
        self.client.Euler(0.1, 0.2, 0.3)  # roll, pitch, yaw
        self.client.BalanceStand()
       
    def TrajectoryFollow(self, target):
        #target = [x, y]
        point_x_temp = target[0] / SPORT_PATH_POINT_SIZE
        point_y_temp = target[1] / SPORT_PATH_POINT_SIZE
        total_time = target[0] / 0.2
        time_temp = total_time / SPORT_PATH_POINT_SIZE
        path = []
        for i in range(SPORT_PATH_POINT_SIZE):

            path_point_tmp = PathPoint(0, 0, 0, 0, 0, 0, 0)

            path_point_tmp.timeFromStart = i * time_temp
            path_point_tmp.x = (
                point_x_temp * i
                + self.px0
            )
            path_point_tmp.y = (
                point_y_temp * i
                + self.py0
            )
            path_point_tmp.yaw = 0
            path_point_tmp.vx = point_x_temp / time_temp
            path_point_tmp.vy = point_y_temp /time_temp
            path_point_tmp.vyaw = 0

            path.append(path_point_tmp)

        self.client.TrajectoryFollow(path)
        return path
            
    def SpecialMotions(self):
        self.client.RecoveryStand()
        print("RecoveryStand !!!")
        time.sleep(1)
        
        self.client.Stretch()
        print("Sit !!!")
        time.sleep(1)  
        
        self.client.RecoveryStand()
        print("RecoveryStand !!!")
        time.sleep(1)


# Robot state
robot_state = unitree_go_msg_dds__SportModeState_()
def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg
    print("Position: ", robot_state.position)


if __name__ == "__main__":

    ChannelFactoryInitialize(0, 'eno1')
        
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    # test = SportModeTest()
    # test.GetInitState(robot_state)

    print("Start test !!!")
    # while True:
    #     test.t += test.dt

    #     test.StandUpDown()
    #     # test.VelocityMove()
    #     # test.BalanceAttitude()
    #     # test.TrajectoryFollow()
    #     # test.SpecialMotions()

    #     time.sleep(test.dt])
    # path = test.TrajectoryFollow([1, 0])
    # for point in path:
    #     print(f'x = {point.x}, y = {point.y}, t = {point.timeFromStart}')