cmake_minimum_required(VERSION 3.8)
project(mrs_sensor_info)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  mrs_lib
  mrs_modules_msgs
  rclcpp
  rclcpp_components
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsSensorInfo_SensorInfo
)

add_library(MrsSensorInfo_SensorInfo SHARED
  src/sensor_info.cpp
)

target_link_libraries(MrsSensorInfo_SensorInfo
  mrs_lib::mrs_lib
  ${mrs_modules_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
)

rclcpp_components_register_nodes(
  MrsSensorInfo_SensorInfo
  "mrs_sensor_info::SensorInfo"
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
