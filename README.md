## JAKA_Unitree_Go2 Base Loongnix

### dependences：
[ROS_NOETIC on Loongarch64](./ros_noetic.md)

[RealSense](https://github.com/IntelRealSense/librealsense)


### 机械臂使用

```python
#引脚定义：
	TX = TX3	#/dev/ttyS2 in loongnix
	RX = RX3	#/dev/ttyS2 in loongnix
	RX_EN_PIN = 50	#enable RX use gpio_50
	TX_EN_PIN = 51	#enable RX use gpio_51
```

```python
import time
import Board	#$WORKSPACE/src/arm_bus_servo/Board.py

while True:
	# 参数：参数1：舵机id; 参数2：位置; 参数3：运行时间
	# 舵机的转动范围0-240度，对应的脉宽为0-1000,即参数2的范围为0-1000

	Board.setBusServoPulse(1, 800, 1000) # 6号舵机转到800位置，用时1000ms
	time.sleep(0.5) # 延时0.5s

	Board.setBusServoPulse(1, 200, 1000) # 6号舵机转到200位置，用时1000ms
	time.sleep(0.5) # 延时0.5s
	# for i in range(10):
	# 	Board.setBusServoPulse(1, 800, 1000) # 6号舵机转到800位置，用时1000ms
	# 	time.sleep(3) # 延时0.5s
	# 	print(i)
```

### Add Joystick (Microsoft X-Box 360 pad)

```bash
#安装noetic-joy *
sudo apt install ros-noetic-joy

# 将接收器插在电脑上，并检查电脑是否发现设备。
ls /dev/input/js*

#测试手柄是否可用
sudo jstest /dev/input/js1

#设置权限，ROS的joy_node需要设备的访问权限
sudo chmod a+rw /dev/input/js1

#配置需要使用的设备，执行该步骤时需要先启动roscore
rosparam set joy_node/dev "/dev/input/js1"

#启动joy_node节点
rosrun joy joy_node

#打开新的终端，查看topic
rostopic echo joy

#启动节点
rosrun joy joy_node _dev_name:="Microsoft X-Box 360 pad"
rosrun go2_controller controller.py 
rosrun go2_controller x_box.py
```

### ROS多机通信
```bash
source ros_master
```