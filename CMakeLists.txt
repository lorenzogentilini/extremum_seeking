cmake_minimum_required(VERSION 3.0.2)
project(extremum_seeking)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES extremum_seeking
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(extremum_seeking_lib
  src/flux_lines.cpp
  src/observer_based.cpp
  src/extremum_seeking_bur.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(extremum_seeking_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(extremum_seeking_node src/extremum_seeking_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(extremum_seeking_node
  ${catkin_LIBRARIES}
  extremum_seeking_lib
)
