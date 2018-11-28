# Install script for directory: /home/zx/test/ws_zx_trans/src/msgs/control_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_msgs/msg" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/BrakeCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/BrakeReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/ECUData.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/EngineReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/GearCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/GearReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/GetECUReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/HMIReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/LampCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/LampReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/ModeCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/ModeReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/SendECUCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/SpeedCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/SpeedReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/SteerCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/SteerReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/ThrottleCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/ThrottleReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/Traj_Node.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/Trajectory.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/VehicleState.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/msg/WheelStateReport.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/control_msgs/catkin_generated/installspace/control_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/include/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/roseus/ros/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/common-lisp/ros/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/gennodejs/ros/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/control_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/control_msgs/catkin_generated/installspace/control_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/control_msgs/catkin_generated/installspace/control_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_msgs/cmake" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/build/msgs/control_msgs/catkin_generated/installspace/control_msgsConfig.cmake"
    "/home/zx/test/ws_zx_trans/build/msgs/control_msgs/catkin_generated/installspace/control_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/control_msgs" TYPE FILE FILES "/home/zx/test/ws_zx_trans/src/msgs/control_msgs/package.xml")
endif()

