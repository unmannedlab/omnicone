#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

add_executable(purePursuit purePursuit.cpp)
add_dependencies(purePursuit ${PROJECT_NAME}_gencfg)
target_link_libraries(purePursuit ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS
  purePursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
