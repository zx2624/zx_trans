cmake_minimum_required(VERSION 2.8.3)
project(vedio_recv)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_driver_msgs
  control_msgs
  detection_result
)

find_package(OpenCV REQUIRED)
find_package(Threads)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS roscpp   std_msgs   sensor_driver_msgs
  control_msgs
  detection_result
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(vedio_recv src/recv.cc)
add_dependencies(vedio_recv sensor_driver_msgs_generate_messages_cpp detection_result_generate_messages_cpp
control_msgs_generate_messages_cpp)
target_link_libraries(vedio_recv ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})


