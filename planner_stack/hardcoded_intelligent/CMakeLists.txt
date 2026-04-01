cmake_minimum_required(VERSION 3.0.2)
project(hardcoded_intelligent)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  plan_handler
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS plan_handler roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hardcoded_intelligent_node
  src/main.cpp
  src/hardcoded_intelligent_node.cpp
)

target_link_libraries(hardcoded_intelligent_node
  ${catkin_LIBRARIES}
)
