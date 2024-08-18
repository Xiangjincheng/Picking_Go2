# 幻尔机械臂SDK

## 目录：

**ArmIK:**_机械臂的逆解代码_

**HiwonderSDK:**_机械臂底层驱动代码_

## 机械臂使用方法：

 根据板子Uart通信接口，修改[BusServoCmd.py](HiwonderSDK/BusServoCmd.py) 

```python
'''
引脚定义：
	TX = TX3	#/dev/ttyS2 in loongnix
	RX = RX3	#/dev/ttyS2 in loongnix
	RX_EN_PIN = 50	#enable RX use gpio_50
	TX_EN_PIN = 51	#enable RX use gpio_51
'''
RX_PIN = 50 #使能引脚
TX_PIN = 51

serialHandle = serial.Serial("/dev/ttyS3", 115200)  # 初始化串口， 波特率为115200
```

```python
#测试代码
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
