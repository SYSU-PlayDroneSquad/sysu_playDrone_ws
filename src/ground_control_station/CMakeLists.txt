cmake_minimum_required(VERSION 3.0.2)
project(ground_control_station)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        dji_sdk         # dji sdk
        image_transport # 融合多topic数据
        message_generation # 自定义消息
        rviz # rviz 插件
        )
find_package(DJIOSDK REQUIRED)
find_package(GeographicLib REQUIRED) #地理库
find_package(Qt5 COMPONENTS Core Widgets REQUIRED) #rviz 插件 Qt5

# 下面两个任选一个
# SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")
add_compile_options(-std=c++11)

# 自定义消息
add_message_files(FILES Array3.msg Status.msg StatusArray.msg StatusNew.msg StatusArrayNew.msg VisualStatus.msg VisualStatusArray.msg)
add_service_files(FILES EnableMotors.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs ground_control_station)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        dji_sdk/dji_sdk/dji_sdk.h
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
)

catkin_package(
#          INCLUDE_DIRS include
#          LIBRARIES ground_control_station_lib
#          CATKIN_DEPENDS roscpp rospy hector_uav_msgs
#          DEPENDS system_lib
)

# 插件包含 Qt 控件，所以如下设置：
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

add_library(
        simulation_control_station_lib
        include/HectorQuadrotor.h
        include/UavGroup.h
        src/simulation_control_station.cpp
)

add_library(
        command_parser_lib
        include/CommandParser.h
        include/DjiController.h
        src/command_parser.cpp
)

add_library(
        ground_control_station_lib
        include/GroundControlStation.h
        src/ground_control_station.cpp
)

add_library(
        GroundControlStation_lib
        include/UavStatusUpdate.h
        include/UavVisualization.h
        include/GroundControlStation.h
        include/UavStatusUpdate.cpp
        include/UavVisualization.cpp
        include/GroundControlStation.cpp
)

# 设置插件源文件
qt5_wrap_cpp(MOC_FILES include/GroundStation2.h )
set(SOURCE_FILES src/ground_station2.cpp ${MOC_FILES})

# 库封装 rviz 插件
add_library(ground_control_station2 ${SOURCE_FILES})
target_link_libraries(ground_control_station2  GroundControlStation_lib ${QT_LIBRARIES} ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})

add_executable(simulation_control_station src/simulation_control_station.cpp)
target_link_libraries(simulation_control_station simulation_control_station_lib ${catkin_LIBRARIES})

add_executable(command_parser src/command_parser.cpp)
target_link_libraries(command_parser command_parser_lib ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_executable(simulation_position_fusion src/simulation_position_fusion.cpp)
target_link_libraries(simulation_position_fusion  ${catkin_LIBRARIES})

add_executable(ground_control_station src/ground_control_station.cpp)
target_link_libraries(ground_control_station   GroundControlStation_lib  ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})



#[[
add_library(
        uav_status_update_lib
        include/UavStatusUpdate.h
        src/uav_status_update.cpp
)

add_executable(uav_status_update src/uav_status_update.cpp)
target_link_libraries(uav_status_update uav_status_update_lib ${catkin_LIBRARIES})
]]
