cmake_minimum_required(VERSION 2.8.3)
project(sensor_driver)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  roslib
  std_msgs
  tf)

find_package(ivcommon REQUIRED)

include("${IVCOMMON_CMAKE_DIR}/functions.cmake")
google_initialize_ivcommon_project()

#google_enable_testing()
#find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
find_package(PCL 1.8 REQUIRED)
#set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV REQUIRED)

find_package(Protobuf REQUIRED)

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

install(FILES package.xml DESTINATION share/sensor_driver)


install(DIRECTORY config DESTINATION share/sensor_driver/)
install(DIRECTORY launch DESTINATION share/sensor_driver/)
install(DIRECTORY rviz_cfg DESTINATION share/sensor_driver/)
install(DIRECTORY urdf DESTINATION share/sensor_driver/)

file(GLOB_RECURSE ALL_UTIL_SRCS "src/util/*")

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_PROTO_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})


include_directories(
	${catkin_INCLUDE_DIRS}
#	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${ivcommon_INCLUDE_DIRS})

list(APPEND LINKER_LIBS ${PROTOBUF_LIBRARY})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp roslib std_msgs
  DEPENDS EIGEN3 PCL OpenCV
)

set(ecu_SRC src/ecu/AnalysisECU.cpp)
set(ins_SRC src/ins/AnalysisINS_Octans.cpp src/ins/AnalysisINS_OxT.cpp src/ins/imu531.cpp)
set(gps_SRC src/gps/AnalysisGPS.cpp)
set(velodyne_SOURCES src/velodyne/myhdl_grabber.cpp)
set(rslidar_SOURCES src/rslidar/myhdl_grabber.cpp)

add_library(utils SHARED ${ALL_UTIL_SRCS}  src/velodyne/HDL32Structure.cpp src/velodyne/roadboundary.cpp ${ALL_PROTO_SRCS})
add_library(ivsensor SHARED ${rslidar_SOURCES}  ${ins_SRC}  ${velodyne_SOURCES}  ${gps_SRC} ${ecu_SRC})


target_link_libraries(ivsensor PUBLIC ivcommon ${catkin_LIBRARIES} pcap)# ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LINKER_LIBS} )
target_link_libraries(utils PUBLIC ivcommon ivsensor )

target_include_directories(ivsensor PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(utils PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(ivsensor PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/src>
)

target_include_directories(utils PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/src>
)

list(APPEND LINKER_LIBS  utils ${catkin_LIBRARIES} ${ivcommon_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LINKER_LIBS}  libgomp.so pcap.so ivsensor)

add_executable(kittidata src/kittidata.cpp)
target_link_libraries(kittidata ${LINKER_LIBS})

add_executable(getvelodynedata src/getvelodynedata.cpp )
target_link_libraries(getvelodynedata  ${LINKER_LIBS})

add_executable(getmutivelodynedata src/getmutivelodynedata.cpp )
target_link_libraries(getmutivelodynedata ${LINKER_LIBS})


add_executable(getrslidardata src/getrslidardata.cpp )
target_link_libraries(getrslidardata ${LINKER_LIBS})

add_executable(getsinglerslidardata src/getsinglerslidardata.cpp )
target_link_libraries(getsinglerslidardata ${LINKER_LIBS})

add_executable(getinsdata src/getinsdata.cpp)
target_link_libraries(getinsdata ${LINKER_LIBS})

add_executable(getgpsdata src/getgpsdata.cpp )
target_link_libraries(getgpsdata ${LINKER_LIBS})

add_executable(getecudata src/getecudata.cpp )
target_link_libraries(getecudata ${LINKER_LIBS})

add_executable(masternode src/masternode.cpp)
target_link_libraries(masternode ${LINKER_LIBS})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -std=c++11")
set_target_properties(getmutivelodynedata getvelodynedata getinsdata PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS masternode getgpsdata getinsdata getrslidardata getsinglerslidardata getmutivelodynedata getvelodynedata kittidata getecudata
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS ivsensor utils masternode getgpsdata getinsdata getrslidardata getsinglerslidardata getmutivelodynedata getvelodynedata kittidata getecudata
  EXPORT sensor_driverExport
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


# Install source headers.
file(GLOB_RECURSE hdrs "*.h" "*.hpp")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)

  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")

foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${PROJECT_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

set(SENSORDRIVER_CMAKE_DIR share/sensor_driver/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  sensor_driver-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/sensor_driver/sensor_driver-config.cmake
  PATH_VARS SENSORDRIVER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/sensor_driver
)

install(
  EXPORT sensor_driverExport
  DESTINATION share/sensor_driver/cmake/
  FILE SensordriverTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/sensor_driver/sensor_driver-config.cmake
  DESTINATION share/sensor_driver/
)

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

