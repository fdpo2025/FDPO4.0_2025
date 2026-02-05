cmake_minimum_required(VERSION 3.0.2)
project(wifi_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wifi_driver_node
  src/main.cpp
  src/wifi_driver_node.cpp
)

add_dependencies(wifi_driver_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(wifi_driver_node
  ${catkin_LIBRARIES}
)
