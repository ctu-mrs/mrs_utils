cmake_minimum_required(VERSION 2.8.3)
project(tf_reconfigure)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  mrs_lib
  dynamic_reconfigure
  tf
  )

find_file(DISTCC_FULL_PATH distcc)
find_file(DISTCPP_FULL_PATH distg++)
IF (EXISTS ${DISTCC_FULL_PATH})
 IF (EXISTS ${DISTCPP_FULL_PATH})
   message(WARNING "Found distcc and distg++, using distributed compilation")
   SET(CMAKE_C_COMPILER ${DISTCC_FULL_PATH})
   SET(CMAKE_CXX_COMPILER ${DISTCPP_FULL_PATH})
 ELSE()
   message(WARNING "Found only distcc, but not distg++, not using distributed compilation")
 ENDIF()
ELSE()
  message(WARNING "Not found distcc, not using distributed compilation")
ENDIF()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2 -fno-diagnostics-color")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -O2 -fno-diagnostics-color")

###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
  config/tf.cfg
  )

catkin_package(
  LIBRARIES TfReconfigure
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp mrs_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

set(LIBS
  ${catkin_LIBRARIES}
  -lboost_system
  )

# Nodelet for OrbSlam (RGBD)
add_library(TfReconfigure
  src/TfReconfigure.cpp
  )

## Add configure headers for dynamic reconfigure
add_dependencies(TfReconfigure ${PROJECT_NAME}_gencfg)

target_link_libraries(TfReconfigure
  ${LIBS}
  )

