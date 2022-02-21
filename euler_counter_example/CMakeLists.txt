cmake_minimum_required(VERSION 3.1.2)
project(euler_counter_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  mrs_lib
  dynamic_reconfigure
  tf
  nodelet
  mrs_lib
  )

generate_dynamic_reconfigure_options(
  config/euler.cfg
  )

catkin_package(
  LIBRARIES EulerCounterExample
  CATKIN_DEPENDS roscpp mrs_lib
  DEPENDS Eigen
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_library(EulerCounterExample
  src/euler_counter_example.cpp
  )

add_dependencies(EulerCounterExample
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  )

target_link_libraries(EulerCounterExample
  ${LIBS}
  ${catkin_LIBRARIES}
  )

