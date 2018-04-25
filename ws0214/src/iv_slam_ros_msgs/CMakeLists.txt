cmake_minimum_required(VERSION 2.8.3)
project(iv_slam_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  roslib
  rospy
  std_msgs
  tf)


add_message_files(
  FILES
  TraversibleArea.msg
)


generate_messages(
DEPENDENCIES
sensor_msgs geometry_msgs
)

