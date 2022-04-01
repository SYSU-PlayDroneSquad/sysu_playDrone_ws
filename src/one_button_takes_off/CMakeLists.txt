cmake_minimum_required(VERSION 3.0.2)
project(one_button_takes_off)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        dji_sdk         # dji sdk
        )
find_package(DJIOSDK REQUIRED)

# 下面两个任选一个
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")
add_compile_options(-std=c++11)

# 自定义消息

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        dji_sdk/dji_sdk/dji_sdk.h
)

catkin_package(
#          INCLUDE_DIRS include
#          LIBRARIES one_button_takes_off_lib
#          CATKIN_DEPENDS roscpp rospy hector_uav_msgs
#          DEPENDS system_lib
)


add_library(
        command_parser_lib
        include/CommandParser.h
        include/DjiController.h
        src/command_parser.cpp
)

add_executable(command_parser src/command_parser.cpp)
target_link_libraries(command_parser command_parser_lib ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})


