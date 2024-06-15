cmake_minimum_required(VERSION 3.0.2)
project(go2_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
  message_generation  # 添加 message_generation 依赖
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go2_controller
  CATKIN_DEPENDS roscpp rospy std_msgs unitree_legged_msgs
  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  /usr/local/include/ddscxx 
  /usr/local/include/iceoryx/v2.0.2
)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)



catkin_install_python(PROGRAMS
  scripts/controller.py
  scripts/x_box.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(ctrl src/ctrl.cpp)
target_link_libraries(ctrl ${catkin_LIBRARIES})
add_dependencies(ctrl unitree_legged_msgs_gencpp)