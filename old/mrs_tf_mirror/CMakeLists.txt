cmake_minimum_required(VERSION 3.5)
project(mrs_tf_mirror)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  message_runtime
  mrs_lib
  nodelet
  roscpp
  tf2
  tf2_ros
  nav_msgs
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(LIBRARIES
  MrsTfMirror_TfMirror
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS Eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsTfMirror_TfMirror
  src/tf_mirror.cpp
  )

add_dependencies(MrsTfMirror_TfMirror
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsTfMirror_TfMirror
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
