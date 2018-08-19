cmake_minimum_required(VERSION 2.8)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

project(myrtt)
find_package(OpenCV REQUIRED)
find_package(Threads)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(client client.cc )
add_executable(server server.cc )
target_link_libraries(client ${OpenCV_LIBS})
target_link_libraries(server ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

