cmake_minimum_required(VERSION 2.8.3)
project(perception_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES perception_utils
 CATKIN_DEPENDS 
 DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(perception_utils
  src/perception_utils.cpp 
  )
target_link_libraries(perception_utils 
  ${catkin_LIBRARIES}
  )


