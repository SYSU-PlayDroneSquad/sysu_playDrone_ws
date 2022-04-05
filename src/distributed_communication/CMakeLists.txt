cmake_minimum_required(VERSION 3.0.2)
project(distributed_communication)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  ground_control_station
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES distributed_communication
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

INCLUDE_DIRECTORIES(/usr/local/include/)
LINK_DIRECTORIES(/usr/local/lib/)

# Find required protobuf package
find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()
#include_directories(${PROTOBUF_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(PROTO_FILE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/distributed_communication)
message(STATUS ${PROTO_FILE_DIR})
#设置protoc的搜索路径
LIST(APPEND PROTO_FLAGS -I${PROTO_FILE_DIR})
#获取需要编译的proto文件
file(GLOB_RECURSE proto_files ${CMAKE_CURRENT_SOURCE_DIR}/include/distributed_communication/*.proto)
set(PROTO_SRCS "")
set(PROTO_HDRS "")
foreach(proto_file ${proto_files})
  get_filename_component(pb_file_name ${proto_file} NAME_WE)#获取pb file文件名

  list(APPEND PROTO_SRCS "${PROTO_FILE_DIR}/${pb_file_name}.pb.cc")
  list(APPEND PROTO_HDRS "${PROTO_FILE_DIR}/${pb_file_name}.pb.h")

  # 使用自定义命令
  # add_custom_command(
  #   OUTPUT "${PROTO_FILE_DIR}/${pb_file_name}.pb.cc"
  #          "${PROTO_FILE_DIR}/${pb_file_name}.pb.h"
  #   COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
  #   ARGS --cpp_out  ${PROTO_FILE_DIR}
  #     -I ${CMAKE_CURRENT_SOURCE_DIR}/include/cloud_server_communication
  #     ${proto_file}
  #   DEPENDS ${proto_file}
  #   COMMENT "Running C++ protocol buffer compiler on ${msg}"
  #   VERBATIM
  # )
  # 生成源码
  execute_process(
    COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_FILE_DIR} ${proto_file}
    )
  
endforeach(proto_file ${proto_files})
# 设置文件属性为 GENERATED
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES GENERATED TRUE)


add_executable(UAV_status_publisher src/UAV_status_publisher.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(UAV_status_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(UAV_status_publisher
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(UAV_status_subscriber src/UAV_status_subscriber.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(UAV_status_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(UAV_status_subscriber
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(test_data_generator src/test_data_generator.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(test_data_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_data_generator
  ${catkin_LIBRARIES}
  protobuf
)

add_executable(GCS_command_publisher src/GCS_command_publisher.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(GCS_command_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GCS_command_publisher
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(GCS_command_subscriber src/GCS_command_subscriber.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(GCS_command_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GCS_command_subscriber
  ${catkin_LIBRARIES}
  protobuf
  zmq
)