cmake_minimum_required(VERSION 3.0.2)
project(topic_broadcaster)

# 编译标准：C++17
add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  scene_graph
  message_runtime
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs scene_graph message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# 本包不定义新消息，无需 add_message_files / generate_messages。
# EncodeMask / WordVector 直接消费 scene_graph 包生成的消息。

add_executable(broadcaster_node
    src/broadcaster_node.cpp
    src/tcp_client.cpp
    src/encodemask_codec.cpp
)
# 依赖 scene_graph 等依赖包生成的消息目标，确保 EncodeMask.h 已生成
add_dependencies(broadcaster_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(broadcaster_node
    ${catkin_LIBRARIES}
    pthread
)

# 安装 launch 与 config，方便 roslaunch / rosrun 使用
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
