cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_msgs)


find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
  std_msgs
#  cmake_utils
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_message_files(
  FILES
  Ellipsoid.msg
  EllipsoidArray.msg
  Polyhedron.msg
  PolyhedronArray.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
#   LIBRARIES encode_msgs decode_msgs
  #CATKIN_DEPENDS geometry_msgs nav_msgs
  #DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)