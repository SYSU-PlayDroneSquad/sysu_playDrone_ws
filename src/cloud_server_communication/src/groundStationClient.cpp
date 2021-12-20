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
#include <std_msgs/String.h>
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
    ROS_INFO("uavIndex: %d", uavIndex);
    ROS_INFO("Ground station client send message (%d bytes) success.",send_byte);
    zmq_msg_close(&send_msg);

    // zmq_msg_t recv_msg;
    // zmq_msg_init(&recv_msg);
    // zmq_msg_recv(&recv_msg,client,0);

    return send_byte;
}

void key_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Send: %s", msg->data.c_str());

    delayMessage::DelayMsg delaymsg;
    delaymsg.set_uav_id(uavIndex);
    delaymsg.set_send_time(ros::Time::now().toSec());
    delaymsg.set_cmd(msg->data.c_str());
    // delaymsg.set_co({0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 32.0, 33.0, 34.0, 35.0});
    // test.set_allocated_ddd(&delaymsg);

    sendMsg(delaymsg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "client_node");
    ros::NodeHandle nh;

    ros::param::get("~uav_name", uavName);
    ros::param::get("~uav_index", uavIndex);

    if(0 == zmq_bind(client, pub_addr)){
        ROS_INFO("Ground station client connect success");//return 0 if success
    }else{
        perror(string("ground station client connect failed:").c_str());
        return -1;
    }

    // if(0 == zmq_connect(client, connect_addr_list[0])){
    //     ROS_INFO("Ground station client connect success");//return 0 if success
    // }else{
    //     perror(string("ground station client connect failed:").c_str());
    //     return -1;
    // }

    // if(0 == zmq_connect(client, server_addr)){
    //     ROS_INFO("%s client connect success", uavName.c_str());//return 0 if success
    // }else{
    //     perror(string(uavName+"client connect failed:").c_str());
    //     return -1;
    // }

    ros::Subscriber keySub = nh.subscribe("key",10,&key_callback);

    delayMessage::DelayMsg delaymsg;
    ros::Rate loop_rate(100);

    ros::spin();
    
    // while(ros::ok()){
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    zmq_close(client);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;

}
