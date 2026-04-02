cmake_minimum_required(VERSION 3.0.2)
project(chris_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Shared source files (planner library logic)
set(PLANNER_LIB_SOURCES
  src/graph.cpp
  src/factory_model.cpp
  src/planner.cpp
  src/warehouse_approach_topology.cpp
)

# chris_planner_node
add_executable(chris_planner_node
  src/chris_planner_main.cpp
  src/chris_planner_node.cpp
  ${PLANNER_LIB_SOURCES}
)
add_dependencies(chris_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(chris_planner_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# multi_planner_node
add_executable(multi_planner_node
  src/multi_planner_main.cpp
  src/multi_planner_node.cpp
  ${PLANNER_LIB_SOURCES}
)
add_dependencies(multi_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_planner_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# comunicacoes_node
add_executable(comunicacoes_node
  src/comunicacoes_main.cpp
  src/comunicacoes_node.cpp
)
add_dependencies(comunicacoes_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(comunicacoes_node
  ${catkin_LIBRARIES}
)

# Install YAML config files
install(DIRECTORY files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/files
  FILES_MATCHING PATTERN "*.yaml"
)
