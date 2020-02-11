cmake_minimum_required(VERSION 2.8.3)
project(roboteq_python)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/roboteq_driver_node
  scripts/set_roboteq_id
  scripts/get_roboteq_id
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

