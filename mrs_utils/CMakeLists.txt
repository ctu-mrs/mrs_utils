cmake_minimum_required(VERSION 3.5)
project(mrs_utils)

set(CATKIN_DEPENDENCIES
    cmake_modules
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )
