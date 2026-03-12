cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(ENABLE_PRECOMPILED_HEADERS "OFF")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED) #to help find Eigen

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  MINCOTraj.msg
  LocalGoal.msg
  SwarmGlobalPathList.msg
  WayPointInfo.msg
  FormationId.msg
  StatusData.msg
  run_base.msg
  kill_base.msg
  node_only_drone_node.msg
  test.msg
  LocalTime.msg
  take_off.msg
  land.msg
  to_drone_state.msg
  check_and_kill_once.msg
  drone_state_to_steamdeck.msg
  CarTraj.msg
  drone_state_to_steamdeck.msg
  LocalTime.msg
  SteamdeckInfo.msg
  drone_die.msg
  YawCmd.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils 
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

# link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  
