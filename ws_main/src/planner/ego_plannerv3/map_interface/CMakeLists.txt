cmake_minimum_required(VERSION 2.8.3)
project(map_interface)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    visualization_msgs
    plan_env
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS plan_env path_searching
 DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

#add_library( map_interface
#     include/map_interface/map_interface.hpp
#     include/map_interface/parameter_server.hpp
#)
#target_link_libraries( map_interface
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#)
