cmake_minimum_required(VERSION 3.0.2)
project(controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
  interfaces
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
  interfaces
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/arm_controller.py
  scripts/unitree_go2.py
  scripts/joystick.py
  scripts/yolov5_pred.py
  scripts/manage.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
