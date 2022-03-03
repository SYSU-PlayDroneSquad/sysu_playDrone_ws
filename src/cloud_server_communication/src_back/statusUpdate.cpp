#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
using namespace std;

string uavName("uav");

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);

    ros::Publisher statusPub = nh.advertise<nav_msgs::Odometry>(uavName+"/delay/status", 10);

    ros::Rate loop_rate(1);

    while(ros::ok()){
        nav_msgs::Odometry status_msg;
        status_msg.pose.pose.position.x=rand()%180;
        status_msg.pose.pose.position.y=rand()%90;;
        status_msg.pose.pose.position.z=rand()%10000;;
        status_msg.twist.twist.linear.x=rand()%10;;
        status_msg.twist.twist.linear.y=rand()%10;;
        status_msg.twist.twist.linear.z=rand()%10;;
        
        statusPub.publish(status_msg);
        ROS_INFO("%s status update", uavName.c_str());

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
