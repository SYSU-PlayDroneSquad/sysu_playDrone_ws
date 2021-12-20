#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <zmq.h>
#include <nav_msgs/Odometry.h>
#include "../include/cloud_server_communication/delay.pb.h"
//#include "../include/delay/delay.pb.cc"
#include "../include/cloud_server_communication/config.h"
#include <std_msgs/String.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

void* ctx = zmq_ctx_new();
void* subscriber = zmq_socket(ctx, ZMQ_SUB);
vector<void*> subscriberList = vector<void*>(51, zmq_socket(ctx, ZMQ_SUB));
string uavName("uav");
int numUav;

ros::Publisher attitudePub;
ros::Publisher positionPub;

delayMessage::DelayMsg delaymsg;
ros::Time stamp;

void pubAttAndGPS(){
    stamp.sec = delaymsg.send_time();
    // assign attitude
    geometry_msgs::QuaternionStamped att_msg;
    att_msg.header.seq      = delaymsg.msg_id();
    att_msg.header.stamp    = stamp;
    att_msg.header.frame_id = delaymsg.str();
    att_msg.quaternion.x    = delaymsg.x();
    att_msg.quaternion.y    = delaymsg.y();
    att_msg.quaternion.z    = delaymsg.z();
    att_msg.quaternion.w    = delaymsg.w();

    // assign GPS_position
    sensor_msgs::NavSatFix pos_msg;
    pos_msg.header.seq      = delaymsg.msg_id();
    pos_msg.header.stamp    = stamp;
    pos_msg.header.frame_id = delaymsg.str();
    pos_msg.latitude        = delaymsg.lat();
    pos_msg.longitude       = delaymsg.lon();
    pos_msg.altitude        = delaymsg.alt();

    attitudePub.publish(att_msg);
    positionPub.publish(pos_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_node");
    ros::NodeHandle nh;
    ros::param::get("~uav_name", uavName);
    ros::param::get("~num_uav", numUav);
    // delayMessage::DelayMsg delaymsg;

    // ros::Publisher odom_pub = nh.advertise<vector<nav_msgs::Odometry>>("odom", 1);

    // publish msg to ground control station
    attitudePub = nh.advertise<geometry_msgs::QuaternionStamped>("attitude", 10);
    positionPub = nh.advertise<sensor_msgs::NavSatFix>("GPS_position", 10);
    ros::Publisher pub = nh.advertise<std_msgs::String>("status_msg", 1000);

    for(int i=0;i<numUav;++i){
        if(0==zmq_connect(subscriberList[i], sub_ground_addr_list[i])){
            ROS_INFO("client %d bind success", i);//return 0 if success
        }
        else{
            perror("server bind failed:");
            return -1;
        }
        zmq_setsockopt(subscriberList[i], ZMQ_SUBSCRIBE, "", 0);
    }

    // if(0==zmq_connect(subscriberList[0], sub_ground_addr_list[0])){
    //     ROS_INFO("client %d bind success", 0);//return 0 if success
    // }
    // else{
    //     perror("server bind failed:");
    //     return -1;
    // }
    // zmq_setsockopt(subscriberList[0], ZMQ_SUBSCRIBE, "", 0);
    
    ros::Rate loop_rate(1);

    while(ros::ok()){
        delaymsg.Clear();
        zmq_msg_t recv_msg;
        zmq_msg_init(&recv_msg);

        for (int i = 0; i < numUav; i++)
        {
            int recv_byte = zmq_msg_recv(&recv_msg,subscriberList[i],0);//ZMQ_DONTWAIT

            if(recv_byte > 0)
            {
                ROS_INFO("Ground control station receive message (%d bytes) success.", recv_byte);

                string str;
                std_msgs::String status;
                str.assign((char*)zmq_msg_data(&recv_msg),recv_byte);
                zmq_msg_close(&recv_msg);
                //    int index = str.find_first_of('\0');
                //    ROS_INFO("%d",index);
                //    delaymsg.ParseFromString(str.substr(0,index));
                delaymsg.ParseFromString(str);
                ROS_INFO("The status of uav%d : %s", delaymsg.uav_id(), delaymsg.cmd().c_str());
                ROS_INFO("The sequence: %d, the stamp: %f, the frame_id: %s", delaymsg.msg_id(), delaymsg.send_time(), delaymsg.str().c_str());
                ROS_INFO("The latitude: %f, the longitude: %f, the altitude: %f", delaymsg.lat(), delaymsg.lon(), delaymsg.alt());
                ROS_INFO("The x: %f, the y: %f, the z: %f, the w: %f", delaymsg.x(), delaymsg.y(), delaymsg.z(), delaymsg.w());
                pubAttAndGPS();
                status.data = delaymsg.cmd();
                pub.publish(status);
            }
        }   

        ros::spinOnce();
        loop_rate.sleep();
    }


    zmq_close(subscriber);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;
}
