#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <zmq.h>
#include "../include/distributed_communication/delay.pb.h"
#include <nav_msgs/Odometry.h>
#include "../include/distributed_communication/config.h"
#include <std_msgs/String.h>

#include "ground_control_station/Array3.h"

using namespace std;

void* ctx = zmq_ctx_new();
void* client = zmq_socket(ctx, ZMQ_PUB);
string uavName("uav");

int uavIndex;

int sendMsg(delayMessage::DelayMsg delaymsg){
    string send_str = delaymsg.SerializeAsString();

    //string转成zmq_msg_t
    zmq_msg_t send_msg;
    zmq_msg_init_size(&send_msg,send_str.length());
    memcpy(zmq_msg_data(&send_msg),send_str.c_str(),send_str.size());

    //发送zmq_msg_t
    int send_byte = zmq_msg_send(&send_msg,client,0);
    ROS_INFO("Ground station client send message (%d bytes) success.",send_byte);
    zmq_msg_close(&send_msg);

    return send_byte;
}

void key_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Send: %s", msg->data.c_str());

    delayMessage::DelayMsg delaymsg;
    delaymsg.set_is_from_keyboard(true);
    delaymsg.set_uav_id(uavIndex);
    delaymsg.set_send_time(ros::Time::now().toSec());
    delaymsg.set_cmd(msg->data.c_str());

    sendMsg(delaymsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ground_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);
    ros::param::get("~uav_index", uavIndex);

    if(0 == zmq_bind(client, GCS_command_pub_addr)){
        ROS_INFO("Ground station client connect success");//return 0 if success
    }else{
        perror(string("ground station client connect failed:").c_str());
        return -1;
    }

    ros::Subscriber keySub = nh.subscribe("key",10,&key_callback);

    ros::spin();

    zmq_close(client);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;
}