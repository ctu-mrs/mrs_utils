cmake_minimum_required(VERSION 3.8)
project(mrs_odometry_republisher)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(DEPENDENCIES
  tf2
  tf2_ros
  mrs_lib
  pluginlib
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  ${mrs_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(MrsOdometryRepublisher_Republisher SHARED 
  src/odometry_republisher.cpp
)

# Apply dependencies to the target
ament_target_dependencies(MrsOdometryRepublisher_Republisher
  ${DEPENDENCIES}
)

# Register the component
rclcpp_components_register_nodes(
  MrsOdometryRepublisher_Republisher
  "odometry_republisher::OdometryRepublisher"
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS MrsOdometryRepublisher_Republisher
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
