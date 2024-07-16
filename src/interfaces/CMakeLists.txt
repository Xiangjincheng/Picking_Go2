cmake_minimum_required(VERSION 3.0.2)
project(interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Rois.msg
  Targets.msg
)

add_service_files(
  FILES
  ArmCtrl.srv
  Depth.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
