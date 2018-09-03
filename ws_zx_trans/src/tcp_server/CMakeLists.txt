cmake_minimum_required(VERSION 2.8.3)
project(tcp_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  detection_result
)

find_package(OpenCV REQUIRED)
find_package(Threads)
catkin_package(
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/tcp_server.cc)


 target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


