project(vedio_send)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  sensor_driver_msgs
  control_msgs
  detection_result
 object_detector_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_udp
#  CATKIN_DEPENDS roscpp  sensor_msgs std_msgs   sensor_driver_msgs
#  control_msgs
#  detection_result
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)



 add_executable(vedio_send src/vedio_send.cpp)
 add_dependencies(vedio_send detection_result_generate_messages_cpp control_msgs_generate_messages_cpp)
 target_link_libraries(vedio_send ${OpenCV_LIBS} ${catkin_LIBRARIES})



