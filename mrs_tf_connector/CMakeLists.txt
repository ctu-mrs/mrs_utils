cmake_minimum_required(VERSION 3.20.0)
project(mrs_tf_connector)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_tf_connector]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  mrs_lib
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_sensor_msgs
  Bullet
  visualization_msgs
)

set(LIBRARIES
  MrsUtils_MRSTFConnector
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

add_library(MrsUtils_MRSTFConnector SHARED
  src/tf_connector.cpp
)

foreach(library IN LISTS LIBRARIES)
  target_link_libraries(${library}
    # ${DEPENDENCIES}
    rclcpp::rclcpp
    rclcpp_components::component
    tf2::tf2
    tf2_ros::tf2_ros
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs
    ${visualization_msgs_TARGETS}
    mrs_lib::mrs_lib
    ${geometry_msgs_TARGETS}
    ${BULLET_LIBRARIES}
  )

  target_include_directories(${library}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${BULLET_INCLUDE_DIRS}
  )
endforeach()

rclcpp_components_register_node(MrsUtils_MRSTFConnector
  PLUGIN "mrs_tf_connector::TFConnector"
  EXECUTABLE mrs_tf_connector_MRSTFConnector
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
