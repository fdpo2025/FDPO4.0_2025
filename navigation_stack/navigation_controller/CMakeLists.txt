cmake_minimum_required(VERSION 3.0.2)
project(navigation_controller)

## -------------------------
## Padrões de compilação
## -------------------------
add_compile_options(-std=c++11)

## -------------------------
## Dependências
## -------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  dynamic_reconfigure
)

## -------------------------
## Dynamic Reconfigure (cfg)
## -------------------------
## Gera headers a partir de cfg/Navigation.cfg
## (Cria navigation_controller/NavigationConfig.h)
generate_dynamic_reconfigure_options(
  cfg/Navigation.cfg
)

## -------------------------
## Mensagens / Serviços
## -------------------------
add_service_files(
  FILES
  NavigationControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## -------------------------
## catkin_package
## -------------------------
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_runtime
    dynamic_reconfigure
)

## -------------------------
## Includes
## -------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## -------------------------
## Executável
## -------------------------
add_executable(navigation_controller_node
  src/main.cpp
  src/navigation_controller_node.cpp
  src/fsm.cpp
)

## Garante que o node só compila após:
## - mensagens/serviços
## - headers do dynamic_reconfigure
add_dependencies(navigation_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(navigation_controller_node
  ${catkin_LIBRARIES}
)

## -------------------------
## Executável State Space (Template)
## -------------------------
add_executable(navigation_controller_ss_node
  src/main_ss.cpp
  src/navigation_controller_ss_node.cpp
)

add_dependencies(navigation_controller_ss_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(navigation_controller_ss_node
  ${catkin_LIBRARIES}
)

## -------------------------
## (Opcional) Instalação
## -------------------------
## Descomenta se quiseres instalar
# install(TARGETS navigation_controller_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch cfg
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN "*.pyc" EXCLUDE
# )
