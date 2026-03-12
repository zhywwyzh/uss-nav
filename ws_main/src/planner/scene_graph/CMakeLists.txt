cmake_minimum_required(VERSION 3.5)
project(scene_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O3 -Wall")

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    nav_msgs
    sensor_msgs
    map_interface
    tf
    message_generation 
    message_runtime
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(igraph 0.10 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_FOUND)
    MESSAGE(STATUS "OpenMP found")
endif()

include_directories(
    SYSTEM
    include
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${IGRAPH_INCLUDE_DIRS}
    include/nlohmann
    libs/libleidenalg/include
)

add_message_files(
  FILES
  WordVector.msg
  EncodeMask.msg
  PromptMsg.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scene_graph
  CATKIN_DEPENDS message_generation message_runtime 
  DEPENDS system_lib
)

# 建立一个文件列表，存放所有需要编译的源文件
set(skelton_cpps
        src/skeleton_generation.cpp
        src/skeleton_astar.cpp
        src/object_factory.cpp
        src/skeleton_cluster.cpp
        src/ikd_Tree.cpp
        src/scene_graph.cpp
        libs/quickhull/QuickHull.cpp

)
set(skeleton_hs
        include/scene_graph/skeleton_generation.h
        include/scene_graph/skeleton_astar.h
        include/scene_graph/object_factory.h
        include/scene_graph/skeleton_cluster.h
        include/scene_graph/ikd_Tree.h
        include/scene_graph/data_structure.h
        include/scene_graph/scene_graph.h
        include/scene_graph/hungarian_alg.h
        include/scene_graph/traj_visualizer.h
)

add_subdirectory(libs/libleidenalg/src)
add_subdirectory(libs/libleidenalg/include)

message(STATUS "===================================================")
message(STATUS "↑↑↑↑↑↑↑ Add Subdirectory: libs/libleidenalg ↑↑↑↑↑↑↑")
message(STATUS "===================================================")

add_library(scene_graph
    ${skeleton_hs}
    ${skelton_cpps}
    ${LEIDENALG_SRC}
    include/nlohmann/json.hpp
)
add_dependencies(scene_graph libleidenalg)
target_link_libraries(scene_graph
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    libleidenalg
)
if(OpenMP_FOUND)
    target_link_libraries(scene_graph ${OpenMP_CXX_LIBRARIES})
    target_compile_options(scene_graph PRIVATE ${OpenMP_CXX_FLAGS})
endif()

add_executable(yolo_recv_test
        libs/quickhull/QuickHull.cpp
        src/ikd_Tree.cpp
        include/scene_graph/ikd_Tree.h
        include/scene_graph/data_structure.h
        src/skeleton_generation.cpp
        src/skeleton_astar.cpp
        include/scene_graph/object_factory.h
        src/object_factory.cpp
        include/scene_graph/skeleton_cluster.h
        src/skeleton_cluster.cpp

        test/obj_test.cpp
)
add_dependencies(yolo_recv_test libleidenalg)
target_link_libraries(yolo_recv_test
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        libleidenalg
)
if(OpenMP_FOUND)
    target_link_libraries(yolo_recv_test ${OpenMP_CXX_LIBRARIES})
    target_compile_options(yolo_recv_test PRIVATE ${OpenMP_CXX_FLAGS})
endif()

add_executable(undistort_node test/image_process_test.cpp)
target_link_libraries(undistort_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_executable(cloud_fov_limit 
    src/cloud_fov_limit.cpp
)
target_link_libraries(cloud_fov_limit
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

#add_executable(cloud_similarity_test
#test/cloud_similarity_test.cpp)
#target_link_libraries(cloud_similarity_test
#        ${catkin_LIBRARIES})
#if(OpenMP_FOUND)
#    target_link_libraries(cloud_similarity_test ${OpenMP_CXX_LIBRARIES})
#    target_compile_options(cloud_similarity_test PRIVATE ${OpenMP_CXX_FLAGS})
#endif()