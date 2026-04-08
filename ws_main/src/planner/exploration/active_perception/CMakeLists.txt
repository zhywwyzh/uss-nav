cmake_minimum_required(VERSION 2.8.3)
project(active_perception)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# The Debug mode will increase the calculate time from 0.03s to 7.0s
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O1 -Wall")

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    quadrotor_msgs
    visualization_msgs
    plan_env
    map_interface
    scene_graph
    path_searching
    cv_bridge
    traj_utils
    lkh_tsp_solver
    perception_utils

)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES active_perception
 CATKIN_DEPENDS plan_env path_searching traj_utils lkh_tsp_solver map_interface perception_utils scene_graph
 DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( active_perception
    include/active_perception/ftr_data_structure.h
    src/ikd_Tree.cpp
    src/viewpoint_handler.cpp
    src/PoseGraph.cpp
    src/frontier_finder.cpp
    src/graph_node.cpp
    src/uniform_grid.cpp
    src/hgrid.cpp
    )
target_link_libraries( active_perception
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)  
