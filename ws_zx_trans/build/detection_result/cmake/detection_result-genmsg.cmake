# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "detection_result: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idetection_result:/home/zx/test/ws_zx_trans/src/detection_result/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(detection_result_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_custom_target(_detection_result_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "detection_result" "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(detection_result
  "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_result
)

### Generating Services

### Generating Module File
_generate_module_cpp(detection_result
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_result
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(detection_result_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(detection_result_generate_messages detection_result_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_dependencies(detection_result_generate_messages_cpp _detection_result_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_result_gencpp)
add_dependencies(detection_result_gencpp detection_result_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_result_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(detection_result
  "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_result
)

### Generating Services

### Generating Module File
_generate_module_eus(detection_result
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_result
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(detection_result_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(detection_result_generate_messages detection_result_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_dependencies(detection_result_generate_messages_eus _detection_result_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_result_geneus)
add_dependencies(detection_result_geneus detection_result_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_result_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(detection_result
  "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_result
)

### Generating Services

### Generating Module File
_generate_module_lisp(detection_result
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_result
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(detection_result_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(detection_result_generate_messages detection_result_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_dependencies(detection_result_generate_messages_lisp _detection_result_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_result_genlisp)
add_dependencies(detection_result_genlisp detection_result_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_result_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(detection_result
  "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_result
)

### Generating Services

### Generating Module File
_generate_module_nodejs(detection_result
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_result
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(detection_result_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(detection_result_generate_messages detection_result_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_dependencies(detection_result_generate_messages_nodejs _detection_result_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_result_gennodejs)
add_dependencies(detection_result_gennodejs detection_result_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_result_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(detection_result
  "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_result
)

### Generating Services

### Generating Module File
_generate_module_py(detection_result
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_result
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(detection_result_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(detection_result_generate_messages detection_result_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zx/test/ws_zx_trans/src/detection_result/msg/detection_result_msg.msg" NAME_WE)
add_dependencies(detection_result_generate_messages_py _detection_result_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(detection_result_genpy)
add_dependencies(detection_result_genpy detection_result_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS detection_result_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_result)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/detection_result
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(detection_result_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(detection_result_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_result)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/detection_result
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(detection_result_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(detection_result_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_result)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/detection_result
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(detection_result_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(detection_result_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_result)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/detection_result
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(detection_result_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(detection_result_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_result)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_result\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/detection_result
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(detection_result_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(detection_result_generate_messages_py std_msgs_generate_messages_py)
endif()
