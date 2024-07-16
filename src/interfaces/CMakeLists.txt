cmake_minimum_required(VERSION 3.0.2)
project(interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
)

add_message_files(
  FILES
  Rois.msg
  Targets.msg
)

add_action_files(
  FILES 
  Go2.action
)

add_service_files(
  FILES
  ArmCtrl.srv
  RoiToPoint.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task
 CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
