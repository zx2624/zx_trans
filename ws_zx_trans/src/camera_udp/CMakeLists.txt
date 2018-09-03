project(camera_udp)
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
  detection_result
)
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_udp
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
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


 add_executable(cmerasend src/sub_one_image.cpp)
 target_link_libraries(cmerasend ${OpenCV_LIBS} ${catkin_LIBRARIES})


