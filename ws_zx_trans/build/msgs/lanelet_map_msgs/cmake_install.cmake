# Install script for directory: /home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zx/test/ws_zx_trans/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lanelet_map_msgs/msg" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/msg/Node.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/msg/Way.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/msg/Regulatory.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/msg/Lanelet.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/msg/LaneletMap.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lanelet_map_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/lanelet_map_msgs/catkin_generated/installspace/lanelet_map_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/include/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/roseus/ros/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/common-lisp/ros/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/gennodejs/ros/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/lanelet_map_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/lanelet_map_msgs/catkin_generated/installspace/lanelet_map_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lanelet_map_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/lanelet_map_msgs/catkin_generated/installspace/lanelet_map_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lanelet_map_msgs/cmake" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/build/msgs/lanelet_map_msgs/catkin_generated/installspace/lanelet_map_msgsConfig.cmake"
    "/home/zx/test/ws_zx_trans/build/msgs/lanelet_map_msgs/catkin_generated/installspace/lanelet_map_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lanelet_map_msgs" TYPE FILE FILES "/home/zx/test/ws_zx_trans/src/msgs/lanelet_map_msgs/package.xml")
endif()

