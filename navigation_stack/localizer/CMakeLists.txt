cmake_minimum_required(VERSION 3.0.2)
project(localizer)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  laser_geometry
)

## Mensagens (antes de catkin_package)
add_message_files(
  FILES
    Pose.msg
    Cluster.msg
    BeaconMatch.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs   # inclui aqui se qualquer .msg usar tipos de sensor_msgs
)

## Agora sim: catkin_package (depois de generate_messages)
catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs nav_msgs sensor_msgs
    tf2 tf2_ros tf2_geometry_msgs message_runtime
    laser_geometry
)

## Eigen (se usares)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Executáveis
add_executable(beacon_detector_node
  src/beacon_detector/main.cpp
  src/beacon_detector/beacon_detector_node.cpp
  src/beacon_detector/dbscan.cpp
  src/general/point.cpp
  src/general/cluster.cpp
)
add_dependencies(beacon_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beacon_detector_node ${catkin_LIBRARIES})

add_executable(localizer_node
  src/ekf_localizer/main.cpp
  src/ekf_localizer/localizer_node.cpp
  src/general/point.cpp
  src/general/cluster.cpp
)
add_dependencies(localizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localizer_node ${catkin_LIBRARIES})

add_executable(mdistortion_compensator_node
  src/motion_distortion_compensator/main.cpp
  src/motion_distortion_compensator/mdistortion_compensator_node.cpp
)
add_dependencies(mdistortion_compensator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mdistortion_compensator_node ${catkin_LIBRARIES})

install(TARGETS beacon_detector_node localizer_node mdistortion_compensator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
