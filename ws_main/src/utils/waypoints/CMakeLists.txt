cmake_minimum_required(VERSION 2.8.3)
project(waypoints)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O0 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  interactive_markers
  nav_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp roslib interactive_markers nav_msgs
#  LIBRARIES 
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(waypoint_node 
               src/waypoint.cpp
               )

target_link_libraries(waypoint_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



