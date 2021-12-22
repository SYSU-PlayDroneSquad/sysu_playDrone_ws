cmake_minimum_required(VERSION 3.0.2)
project(cloud_server_communication)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  rospy
  image_transport
  ground_control_station
  #protobuf REQUIRED PATHS /usr/local/lib/libprotobuf.a
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
#   std_msgs  # Or other packages containing msgs
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
#  LIBRARIES cloud_server_communication
#  CATKIN_DEPENDS roscpp rosmsg rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

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

set(PROTO_FILE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/cloud_server_communication)
message(STATUS ${PROTO_FILE_DIR})
#设置protoc的搜索路径
LIST(APPEND PROTO_FLAGS -I${PROTO_FILE_DIR})
#获取需要编译的proto文件
file(GLOB_RECURSE proto_files ${CMAKE_CURRENT_SOURCE_DIR}/include/cloud_server_communication/*.proto)
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

# 添加自定义target
# add_custom_target(generate_message ALL
#                 DEPENDS ${PROTO_SRCS} ${PROTO_HDRS}
#                 COMMENT "generate message target"
#                 VERBATIM
#                 )


add_executable(client2ground src/client2ground.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(client2ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(client2ground
  ${catkin_LIBRARIES}
  protobuf
  zmq
)


add_executable(subscribeGround src/subscribeGround.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(subscribeGround ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribeGround
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(subscribe src/subscribe.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(subscribe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(subscribe
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(statusUpdate src/statusUpdate.cpp)
add_dependencies(statusUpdate ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(statusUpdate
  ${catkin_LIBRARIES}
)

add_executable(delayClient src/delayClient.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(delayClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(delayClient
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(cloudServer src/cloudServer.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(cloudServer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cloudServer
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(cloudServerMultithread src/cloudServerMultithread.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(cloudServerMultithread ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(cloudServerMultithread
  ${catkin_LIBRARIES}
  protobuf
  zmq
)

add_executable(groundStationClient src/groundStationClient.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_dependencies(groundStationClient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(groundStationClient
  ${catkin_LIBRARIES}
  protobuf
  zmq
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/delay.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/delay_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_delay.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
