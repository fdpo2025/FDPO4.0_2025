cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_planner_node
  src/path_planner_node.cpp
  src/path_planner.cpp
)

target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
)
