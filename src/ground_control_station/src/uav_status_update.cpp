// ROS include
#include <ros/ros.h>
#include <std_msgs/String.h>

// customer include
#include "UavStatusUpdate.h"

void statusSubCB(const std_msgs::String::ConstPtr& msg, UavStatusUpdate update){
    // update.data_handing(若干参数);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "updateUavStatus");
    ros::NodeHandle nh;
    int uavNum;
    nh.getParam("/uav_status_update/uavNum", uavNum);

    UavStatusUpdate uav_status_update(nh, uavNum);

    // 初始化 UavStatusUpdate::_status
    vector<vector<string>> status_init(uavNum);
    for(int i = 0; i < uavNum; i++){
        status_init[i].emplace_back("");
        status_init[i].emplace_back("0");
        status_init[i].emplace_back(" ");
    }
    UavStatusUpdate::_status = status_init;
    cout << "status init success!" << endl;

    ros::Subscriber statusSub = nh.subscribe<std_msgs::String>(
            "/uav1/status", 10, boost::bind(&statusSubCB, _1, uav_status_update));

    ros::Timer check_status =
            nh.createTimer(ros::Duration(5), &UavStatusUpdate::check_status_CB, &uav_status_update);
    ros::spin();
}