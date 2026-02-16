cmake_minimum_required(VERSION 3.0.2)
project(A2B_Planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/a2b_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)