cmake_minimum_required(VERSION 3.1.2)
project(tf_reconfigure)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  mrs_lib
  dynamic_reconfigure
  tf
  )

generate_dynamic_reconfigure_options(
  config/tf.cfg
  )

catkin_package(
  LIBRARIES TfReconfigure
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

add_dependencies(TfReconfigure ${PROJECT_NAME}_gencfg)

target_link_libraries(TfReconfigure
  ${LIBS}
  )

