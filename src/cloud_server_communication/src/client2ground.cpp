#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <zmq.h>
#include "../include/cloud_server_communication/delay.pb.h"
//#include "../include/delay/delay.pb.cc"
#include <nav_msgs/Odometry.h>
#include "../include/cloud_server_communication/config.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>

// include message_filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace message_filters;

void* ctx = zmq_ctx_new();
void* ground = zmq_socket(ctx, ZMQ_PUB);
string uavName("uav");
delayMessage::DelayMsg delaymsg;

int uavIndex;

int sendMsg(delayMessage::DelayMsg delaymsg){
    string send_str = delaymsg.SerializeAsString();

    //string转成zmq_msg_t
    zmq_msg_t send_msg;
    zmq_msg_init_size(&send_msg,send_str.length());
    memcpy(zmq_msg_data(&send_msg),send_str.c_str(),send_str.size());
    
    //发送zmq_msg_t
    int send_byte = zmq_msg_send(&send_msg,ground,0);
    ROS_INFO("%s client send message (%d bytes) success.",uavName.c_str(), send_byte);
    zmq_msg_close(&send_msg);

    // zmq_msg_t recv_msg;
    // zmq_msg_init(&recv_msg);
    // zmq_msg_recv(&recv_msg,client,0);

    return send_byte;
}

void health_callback(const std_msgs::UInt8::ConstPtr& health){
    delaymsg.set_gps(health->data);
}

void odometryCallback(const geometry_msgs::QuaternionStamped::ConstPtr& att, const sensor_msgs::NavSatFix::ConstPtr& pos){  
    delaymsg.set_uav_id(uavIndex);

    // set header
    delaymsg.set_msg_id(pos->header.seq);
    delaymsg.set_send_time(ros::Time::now().toSec());
    delaymsg.set_str(pos->header.frame_id);

    // set attitude
    delaymsg.set_x(att->quaternion.x);
    delaymsg.set_y(att->quaternion.y);
    delaymsg.set_z(att->quaternion.z);
    delaymsg.set_w(att->quaternion.w);

    // set GPS_position
    delaymsg.set_lat(pos->latitude);
    delaymsg.set_lon(pos->longitude);
    delaymsg.set_alt(pos->altitude);

    sendMsg(delaymsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);
    ros::param::get("~uav_index", uavIndex);

    // communicate with clients
    if(0 == zmq_bind(ground, pub_ground_addr)){
        ROS_INFO("%s client connect success", uavName.c_str());//return 0 if success
    }else{
        perror(string(uavName+"client connect failed:").c_str());
        return -1;
    }

    // if(0 == zmq_connect(client, server_addr)){
    //     ROS_INFO("%s client connect success", uavName.c_str());//return 0 if success
    // }else{
    //     perror(string(uavName+"client connect failed:").c_str());
    //     return -1;
    // }

    ros::Subscriber healthSub = nh.subscribe("GPS_health",10,&health_callback);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(nh, "attitude", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> position_sub(nh, "GPS_position", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::QuaternionStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), attitude_sub, position_sub);
    sync.registerCallback(boost::bind(&odometryCallback, _1, _2));

    ros::Rate loop_rate(1);
    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    zmq_close(ground);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;

}
