import time
import Board

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