cmake_minimum_required(VERSION 3.1.2)
project(tf_reconfigure)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  mrs_lib
  dynamic_reconfigure
  tf
  nodelet
  )

generate_dynamic_reconfigure_options(
  config/tf.cfg
  )

set(LIBRARIES
  TfReconfigure
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp mrs_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(TfReconfigure
  src/TfReconfigure.cpp
  )

add_dependencies(TfReconfigure
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(TfReconfigure
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

install(DIRECTORY launch config rviz start
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
