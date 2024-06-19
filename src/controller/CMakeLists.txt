cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
  interfaces_msgs
  message_generation  
)


catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES controller
  #DEPENDS system_lib
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  unitree_legged_msgs  
  interfaces_msgs

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS
  scripts/unitree_go2.py
  scripts/joystick.py
  scripts/yolov5_pred.py
  scripts/jaka_controller.py
  scripts/camera_realsense.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
