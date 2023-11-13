cmake_minimum_required(VERSION 3.5)
project(mrs_tf_connector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIS
  cmake_modules
  ddynamic_reconfigure
  dynamic_reconfigure
  mrs_lib
  nodelet
  roscpp
  tf2
  tf2_ros
  tf2_sensor_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIS}
  )

set(LIBRARIES
  MrsUtils_MRSTFConnector
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIS}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsUtils_MRSTFConnector
  src/tf_connector.cpp
  )

add_dependencies(MrsUtils_MRSTFConnector
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsUtils_MRSTFConnector
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

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
