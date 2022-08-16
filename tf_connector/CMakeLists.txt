cmake_minimum_required(VERSION 3.1.2)
project(tf_connector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_lib
  dynamic_reconfigure
  tf2
  tf2_ros
  nodelet
  ddynamic_reconfigure
  )

set(LIBRARIES
  TFConnector
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp nodelet mrs_lib dynamic_reconfigure ddynamic_reconfigure tf2 tf2_ros
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(TFConnector
  src/tf_connector.cpp
  )

add_dependencies(TFConnector
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TFConnector
  ${LIBS}
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
