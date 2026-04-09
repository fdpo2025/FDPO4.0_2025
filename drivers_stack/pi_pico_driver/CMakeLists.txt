cmake_minimum_required(VERSION 3.0.2)
project(pi_pico_driver)

## Compilar com C++11
add_compile_options(-std=c++11)

## Encontrar pacotes ROS necessários
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  RadioNetworkTable.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declarar o pacote catkin
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime
)

## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## =========================
##  NÓ: pico_driver_node
## =========================
add_executable(pico_driver_node
  src/main.cpp
  src/pico_driver_node.cpp
)

target_link_libraries(pico_driver_node
  ${catkin_LIBRARIES}
)

add_dependencies(pico_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
