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

add_executable(wifi_driver_node
  src/main.cpp
  src/wifi_driver_node.cpp
)

target_link_libraries(wifi_driver_node
  ${catkin_LIBRARIES}
)
