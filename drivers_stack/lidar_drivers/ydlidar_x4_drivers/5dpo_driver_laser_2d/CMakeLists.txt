cmake_minimum_required(VERSION 3.0.2)
project(sdpo_driver_laser_2d)

add_compile_options(-std=c++11)

add_subdirectory(third_party/YDLidar-SDK)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  sdpo_ros_serial_port
  sensor_msgs
  tf
)

generate_dynamic_reconfigure_options(
  config/sdpo_driver_laser_2d.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_YDLIDARX4
    ${PROJECT_NAME}_YDLIDARX4_SDK
    ${PROJECT_NAME}_RPLIDARS2
  CATKIN_DEPENDS
    roscpp
    rospy
    dynamic_reconfigure
    sdpo_ros_serial_port
    sensor_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YDLidar-SDK
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YDLidar-SDK/src
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/YDLidar-SDK/core
)

# -----------------------
# Libraries (drivers)
# -----------------------

add_library(${PROJECT_NAME}_YDLIDARX4
  src/${PROJECT_NAME}/SdpoDriverLaser2D.cpp
  src/${PROJECT_NAME}/YDLIDARX4.cpp
)

add_library(${PROJECT_NAME}_YDLIDARX4_SDK
  src/${PROJECT_NAME}/SdpoDriverLaser2D.cpp
  src/${PROJECT_NAME}/YDLIDARX4_SDK.cpp
)

add_library(${PROJECT_NAME}_RPLIDARS2
  src/${PROJECT_NAME}/SdpoDriverLaser2D.cpp
  src/${PROJECT_NAME}/RPLIDARS2.cpp
)

add_dependencies(${PROJECT_NAME}_YDLIDARX4
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_YDLIDARX4_SDK
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_RPLIDARS2
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

# Link do SDK só no driver SDK (e no antigo se estiveres mesmo a usar SDK lá)
target_link_libraries(${PROJECT_NAME}_YDLIDARX4_SDK
  ${catkin_LIBRARIES}
  ydlidar_sdk
  pthread
)

target_link_libraries(${PROJECT_NAME}_YDLIDARX4
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_RPLIDARS2
  ${catkin_LIBRARIES}
)

# -----------------------
# Main node (ROS)
# -----------------------

add_executable(${PROJECT_NAME}_node
  src/sdpo_driver_laser_2d/SdpoDriverLaser2DROS.cpp
  src/main.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

# O node principal linka com as libs dos drivers
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_YDLIDARX4_SDK
  ${PROJECT_NAME}_YDLIDARX4
  ${PROJECT_NAME}_RPLIDARS2
)

# -----------------------
# YDLIDAR X4 SDK standalone node
# -----------------------
add_executable(ydlidar_x4_sdk_node
  src/sdpo_driver_laser_2d/ydlidar_x4_sdk_node.cpp
)

add_dependencies(ydlidar_x4_sdk_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ydlidar_x4_sdk_node
  ${catkin_LIBRARIES}
  ydlidar_sdk
  pthread
)
