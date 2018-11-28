# Install script for directory: /home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbw_mkz_msgs/msg" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/AmbientLight.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/BrakeCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/BrakeInfoReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/BrakeReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/FuelLevelReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/Gear.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/GearCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/GearReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/HillStartAssist.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/Misc1Report.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/ParkingBrake.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/SteeringCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/SteeringReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/SurroundReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/SuspensionReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/ThrottleCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/ThrottleInfoReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/ThrottleReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/TirePressureReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/TurnSignal.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/TurnSignalCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/TwistCmd.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/WatchdogCounter.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/WheelSpeedReport.msg"
    "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/msg/Wiper.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbw_mkz_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/dbw_mkz_msgs/catkin_generated/installspace/dbw_mkz_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/include/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/roseus/ros/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/common-lisp/ros/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/share/gennodejs/ros/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zx/test/ws_zx_trans/devel/lib/python2.7/dist-packages/dbw_mkz_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/dbw_mkz_msgs/catkin_generated/installspace/dbw_mkz_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbw_mkz_msgs/cmake" TYPE FILE FILES "/home/zx/test/ws_zx_trans/build/msgs/dbw_mkz_msgs/catkin_generated/installspace/dbw_mkz_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbw_mkz_msgs/cmake" TYPE FILE FILES
    "/home/zx/test/ws_zx_trans/build/msgs/dbw_mkz_msgs/catkin_generated/installspace/dbw_mkz_msgsConfig.cmake"
    "/home/zx/test/ws_zx_trans/build/msgs/dbw_mkz_msgs/catkin_generated/installspace/dbw_mkz_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dbw_mkz_msgs" TYPE FILE FILES "/home/zx/test/ws_zx_trans/src/msgs/dbw_mkz_msgs/package.xml")
endif()

