## JAKA_Unitree_Go2 Base Loongnix 

### dependences：
[ROS_NOETIC on Loongarch64](./ros_noetic.md)




### add Joystick (Microsoft X-Box 360 pad)

```bash
#安装noetic-joy
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