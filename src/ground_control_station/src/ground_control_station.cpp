// ros include
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>


// other include
#include <GeographicLib/LocalCartesian.hpp>
#include<iomanip>

// customer include
#include "uav_visualization.h"
#include "ground_control_station/Status.h"
#include "ground_control_station/StatusArray.h"

using std::cout;
using std::endl;
using std::string;
using std::left;
using std::right;
using std::setw; // 对齐
using std::setprecision; // 精确


// 订阅 dji_sdk 发布的 gps 坐标
void status_sub_CB(const ground_control_station::StatusArrayConstPtr& status){

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_marker");
    ros::NodeHandle nh;
    string shape_frame = "十";
    nh.getParam("/shape_frame", shape_frame);
    UavMarker uavMarker(nh, shape_frame);

    ros::Subscriber status_sub = nh.subscribe<ground_control_station::StatusArray>
            ("/uav1/dji_sdk/gps_position", 10, boost::bind(status_sub_CB, _1));

    ros::Timer revolutions_per_second = nh.createTimer(ros::Duration(0.02), &UavMarker::rps_CB, &uavMarker);

//    UavMarker::pub_frame();

    ros::spin();
}
