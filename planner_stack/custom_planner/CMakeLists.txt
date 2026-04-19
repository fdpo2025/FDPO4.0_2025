cmake_minimum_required(VERSION 3.0.2)
project(custom_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  pi_pico_driver
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs pi_pico_driver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(custom_planner_node
  src/main.cpp
  src/custom_planner_node.cpp
)

target_link_libraries(custom_planner_node
  ${catkin_LIBRARIES}
)

add_dependencies(custom_planner_node ${catkin_EXPORTED_TARGETS} pi_pico_driver_generate_messages_cpp)
