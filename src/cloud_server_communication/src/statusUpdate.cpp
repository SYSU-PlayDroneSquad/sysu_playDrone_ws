#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include "std_msgs/String.h" 
#include <std_msgs/UInt8.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

string uavName("uav");

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);

    ros::Publisher attitudePub = nh.advertise<geometry_msgs::QuaternionStamped>("attitude", 10);
    ros::Publisher positionPub = nh.advertise<sensor_msgs::NavSatFix>("GPS_position", 10);

    // ros::Rate loop_rate(1);

    // while(ros::ok()){
    //     nav_msgs::Odometry status_msg;
    //     status_msg.pose.pose.position.x=rand()%180;
    //     status_msg.pose.pose.position.y=rand()%90;
    //     status_msg.pose.pose.position.z=rand()%10000;
    //     status_msg.twist.twist.linear.x=rand()%10;
    //     status_msg.twist.twist.linear.y=rand()%10;
    //     status_msg.twist.twist.linear.z=rand()%10;
        
    //     statusPub.publish(status_msg);
    //     ROS_INFO("%s status update", uavName.c_str());

    //     loop_rate.sleep();
    // }

    ros::Publisher healthPub = nh.advertise<std_msgs::UInt8>(uavName+"/GPS_health", 10);

    ros::Rate loop_rate(1);

    while(ros::ok()){
        // std_msgs::String state_msg;

        // std::stringstream ss;
        // ss << "OK ";
        // state_msg.data = ss.str();
        
        // statePub.publish(state_msg);

        // gps_health initialization
        std_msgs::UInt8 health;
        health.data = rand()%5;

        healthPub.publish(health);

        // attitude initialization
        geometry_msgs::QuaternionStamped att;
        att.quaternion.x = rand()%3;
        att.quaternion.y = rand()%3;
        att.quaternion.z = rand()%3;
        att.quaternion.w = rand()%3;

        // GPS_position initialization
        sensor_msgs::NavSatFix pos;
        pos.latitude = rand()%1000;
        pos.longitude = rand()%1000;
        pos.altitude = rand()%100;

        positionPub.publish(pos);
        attitudePub.publish(att);

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}