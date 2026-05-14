# Install script for directory: /home/nv/L_ws_lsy/n1_L_ego/src/faster-lio

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/noetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "None")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio/msg" TYPE FILE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/msg/Pose6D.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio/cmake" TYPE FILE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/catkin_generated/installspace/faster_lio-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/include/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/share/roseus/ros/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/share/common-lisp/ros/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/share/gennodejs/ros/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib/python3/dist-packages/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib/python3/dist-packages/faster_lio")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/catkin_generated/installspace/faster_lio.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio/cmake" TYPE FILE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/catkin_generated/installspace/faster_lio-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio/cmake" TYPE FILE FILES
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/catkin_generated/installspace/faster_lioConfig.cmake"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/catkin_generated/installspace/faster_lioConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio" TYPE FILE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/faster_lio" TYPE EXECUTABLE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib/faster_lio/run_mapping_online")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online"
         OLD_RPATH "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_online")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/faster_lio" TYPE EXECUTABLE FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib/faster_lio/run_mapping_offline")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline"
         OLD_RPATH "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib:/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/faster_lio/run_mapping_offline")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/devel/lib/libfaster_lio.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfaster_lio.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/faster_lio" TYPE DIRECTORY FILES
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/cmake"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/config"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/docker"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/launch"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/result"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/rviz_cfg"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/scripts"
    "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/thirdparty"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/gtest/cmake_install.cmake")
  include("/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/src/cmake_install.cmake")
  include("/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/app/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nv/L_ws_lsy/n1_L_ego/src/faster-lio/.obj-aarch64-linux-gnu/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
