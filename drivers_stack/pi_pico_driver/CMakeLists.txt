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
  plan_handler
  tf
)

## Declarar o pacote catkin
catkin_package()

## Incluir diretórios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## =========================
##  NÓ: pico_driver_node
## =========================

## Caso uses um ficheiro principal separado (recomendado)
##   src/pico_driver_main.cpp  → contém o main()
##   src/pico_driver_node.cpp  → contém a classe PiPicoDriver
add_executable(pico_driver_node
  src/main.cpp
  src/pico_driver_node.cpp
)

## Se ainda não criaste o pico_driver_main.cpp (com o main()),
## substitui a linha acima por esta:
# add_executable(pico_driver_node src/pico_driver_node.cpp)

## Linkar bibliotecas ROS
target_link_libraries(pico_driver_node
  ${catkin_LIBRARIES}
)

## Dependências de build (mensagens, etc.)
add_dependencies(pico_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
