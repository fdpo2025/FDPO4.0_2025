cmake_minimum_required(VERSION 3.0.2)
project(chris_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/chris_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install modules directory as a Python package
install(DIRECTORY modules/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install YAML files
install(DIRECTORY files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/files
  FILES_MATCHING PATTERN "*.yaml"
)

