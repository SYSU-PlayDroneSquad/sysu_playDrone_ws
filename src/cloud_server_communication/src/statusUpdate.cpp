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

#include "ground_control_station/Array3.h"

using namespace std;

string uavName("uav");

int main(int argc, char** argv){
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);

    ros::Publisher attitudePub = nh.advertise<geometry_msgs::QuaternionStamped>("attitude", 10);
    ros::Publisher positionPub = nh.advertise<sensor_msgs::NavSatFix>("GPS_position", 10);
    ros::Publisher velocityPub = nh.advertise<ground_control_station::Array3>("vel_list", 10);
    ros::Publisher healthPub = nh.advertise<std_msgs::UInt8>("GPS_health", 10);
    ros::Publisher flightStatusPub = nh.advertise<std_msgs::UInt8>("flight_status", 10);
    ros::Publisher targetPub = nh.advertise<sensor_msgs::NavSatFix>("target_position", 10);

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

        // flight status initialization
        std_msgs::UInt8 flight_status;
        flight_status.data = rand()%3;

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

        // target_position initialization
        sensor_msgs::NavSatFix target_pos;
        target_pos.latitude = rand()%1000;
        target_pos.longitude = rand()%1000;
        target_pos.altitude = rand()%100;

        targetPub.publish(target_pos);

        // // vel_list initialization
        // ground_control_station::Array3 vel;
        // vector<double> tmp(10, 8);
        // tmp[0] = 6;
        // vel.x = tmp;
        // tmp[1] = 18;
        // vel.y = tmp;
        // tmp[2] = 88;
        // vel.z = tmp;

        // velocityPub.publish(vel);

        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}