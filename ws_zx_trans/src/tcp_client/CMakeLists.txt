cmake_minimum_required(VERSION 2.8.3)
project(tcp_client)

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
  sensor_driver_msgs
  control_msgs
  detection_result
)

find_package(OpenCV REQUIRED)
find_package(Threads)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tcp_client
  CATKIN_DEPENDS roscpp   std_msgs   sensor_driver_msgs
  control_msgs
  detection_result
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/tcp_client.cc)
 add_dependencies(${PROJECT_NAME}_node detection_result_generate_messages_cpp control_msgs_generate_messages_cpp)
 target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tcp_client.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
