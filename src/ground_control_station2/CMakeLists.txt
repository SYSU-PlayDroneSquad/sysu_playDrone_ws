cmake_minimum_required(VERSION 3.0.2)
project(ground_control_station2)


find_package(catkin REQUIRED COMPONENTS
        ground_control_station
        roscpp
        rospy
        rviz
        std_msgs
)
add_compile_options(-std=c++11)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ground_control_station2
#  CATKIN_DEPENDS ground_control_station roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# 插件包含 Qt 控件，所以如下设置：
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
        include/GroundStation2.h
        )

set(SOURCE_FILES
        src/ground_station2.cpp
        ${MOC_FILES}
        )


add_library(${PROJECT_NAME} ${SOURCE_FILES})

# 将库与上面 find_package(Qt4 ...) 行定义的任何 Qt 库以及 catkin 包含的任何库链接。
# 尽管这将 “learn_rviz_plugin”（或您所称的项目）作为库的名称，但 cmake 知道它是一个库并将实际文件命名为
# “learn_rviz_plugin.so”之类的名称，或者任何适合您特定操作系统的名称。
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})