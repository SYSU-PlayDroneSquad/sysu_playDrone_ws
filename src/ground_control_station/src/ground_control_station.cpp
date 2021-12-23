// ros include
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>


// other include
#include <GeographicLib/LocalCartesian.hpp>
#include<iomanip>

// customer include
#include "GroundControlStation.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_marker");
    ros::NodeHandle nh;

    // 从参数服务器获取参数
    int uavNumbers(8);
    string shape_frame("+");
    double origin_latitude , origin_longitude, origin_altitude;
    nh.getParam("/uavNumbers", uavNumbers);
    nh.getParam("/shape_frame", shape_frame);
    nh.param("/origin_latitude", origin_latitude, 23.0659742859);
    nh.param("/origin_longitude", origin_longitude, 113.383032051);
    nh.param("/origin_altitude", origin_altitude, -81.517552681);

    // 实例化
    GroundControlStation groundControlStation(nh, uavNumbers, origin_latitude, origin_longitude, origin_altitude);

    ros::spin();
}
