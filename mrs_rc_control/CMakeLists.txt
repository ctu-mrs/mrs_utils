cmake_minimum_required(VERSION 3.1.2)
project(mrs_rc_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  std_srvs
  mavros_msgs
  topic_tools
  )

set(LIBRARIES
  RcTrajectoryTrackingTrigger
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib roscpp std_srvs 
  )

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  )

add_library(RcTrajectoryTrackingTrigger
  src/rc_trajectory_tracking_trigger.cpp)

add_dependencies(RcTrajectoryTrackingTrigger
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(RcTrajectoryTrackingTrigger
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

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
