cmake_minimum_required(VERSION 3.5)
project(mrs_sensor_info)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  message_runtime
  mrs_lib
  mrs_modules_msgs
  nodelet
  roscpp
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(LIBRARIES
  MrsSensorInfo_SensorInfo
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(MrsSensorInfo_SensorInfo
  src/sensor_info.cpp
  )

add_dependencies(MrsSensorInfo_SensorInfo
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MrsSensorInfo_SensorInfo
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

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
