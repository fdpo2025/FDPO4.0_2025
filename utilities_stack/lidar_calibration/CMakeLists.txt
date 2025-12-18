cmake_minimum_required(VERSION 3.0.2)
project(lidar_calibration)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  localizer
  message_generation
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# Serviços
add_service_files(
  FILES
  MeasurementCollection.srv
  GetStatistics.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  std_srvs
  localizer
  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

# Nó de recolha
add_executable(measurement_collector_node src/measurement_collector_node.cpp)
target_link_libraries(measurement_collector_node ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(measurement_collector_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

# Nó de cálculo de estatísticas
add_executable(statistics_calculator_node src/statistics_calculator_node.cpp)
target_link_libraries(statistics_calculator_node ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(statistics_calculator_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)


