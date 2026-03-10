cmake_minimum_required(VERSION 3.20)
project(mrs_tf_mirror)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_tf_mirror]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

set(DEPENDENCIES
  mrs_lib
  mrs_msgs
  mrs_modules_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_sensor_msgs
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(LIBRARIES
  MrsTfMirror_TfMirror
)

add_library(MrsTfMirror_TfMirror SHARED
  src/tf_mirror.cpp
)

ament_target_dependencies(MrsTfMirror_TfMirror
  ${DEPENDENCIES}
)

target_include_directories(MrsTfMirror_TfMirror
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(MrsTfMirror_TfMirror
  PLUGIN "mrs_tf_mirror::TfMirror"
  EXECUTABLE tf_mirror_node
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
