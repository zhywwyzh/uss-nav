cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -Wall")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  traj_utils
  active_perception
  # lkh_tsp_solver
  cv_bridge
  ego_planner
  mavros
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS active_perception traj_utils ego_planner mavros # lkh_tsp_solver
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(exploration_node
  src/exploration_node.cpp 
  src/fast_exploration_fsm.cpp
  src/frontier_manager.cpp
#        src/perception_data_factory.cpp
)
add_dependencies(exploration_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(exploration_node 
  ${catkin_LIBRARIES}
  # -ldw
  )

add_executable(planner_cmd_mux
  src/planner_cmd_mux.cpp
)
add_dependencies(planner_cmd_mux ${catkin_EXPORTED_TARGETS})

target_link_libraries(planner_cmd_mux
  ${catkin_LIBRARIES}
)
