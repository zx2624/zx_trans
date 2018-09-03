cmake_minimum_required(VERSION 2.8.3)
project(vedio_recv)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Threads)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(vedio_recv src/recv.cc)
target_link_libraries(vedio_recv ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES})


