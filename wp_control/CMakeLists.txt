cmake_minimum_required(VERSION 2.8.3)
project(wp_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  geodesy
  sensor_msgs
  std_msgs
  tf
  pacmod_msgs
  novatel_msgs
)

find_package(Boost 1.49.0 REQUIRED)
find_package(Eigen3 REQUIRED)

set(BUILD_FLAGS "-std=c++11 -Wuninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

generate_dynamic_reconfigure_options(cfg/gpsWaypoint_params.cfg)

catkin_package(
  DEPENDS Boost
  CATKIN-DEPENDS roscpp std_msgs geometry_msgs geodesy novatel_msgs nav_msgs pacmod dynamic_reconfigure
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${Eigen3_INCLUDE_DIRS})

add_subdirectory(src/gpsWaypoint)
add_subdirectory(src/purePursuit)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

#add_executable(GenerateWaypoints src/GenerateWaypoints)
