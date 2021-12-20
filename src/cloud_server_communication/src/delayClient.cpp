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
    ROS_INFO("%s client send message (%d bytes) success.",uavName.c_str(), send_byte);
    zmq_msg_close(&send_msg);

    // zmq_msg_t recv_msg;
    // zmq_msg_init(&recv_msg);
    // zmq_msg_recv(&recv_msg,client,0);

    return send_byte;
}

void status_callback(const nav_msgs::Odometry& status){
    delayMessage::DelayMsg delaymsg;
    // delayMessage::Test test;
    delaymsg.set_uav_id(uavIndex);
    delaymsg.set_send_time(ros::Time::now().toSec());
    delaymsg.set_lat(status.pose.pose.position.x);
    delaymsg.set_lon(status.pose.pose.position.y);
    delaymsg.set_alt(status.pose.pose.position.z);
    delaymsg.set_vx(status.twist.twist.linear.x);
    delaymsg.set_vy(status.twist.twist.linear.y);
    delaymsg.set_vz(status.twist.twist.linear.z);

    // delaymsg.set_xo(8.8);
    // delaymsg.set_yo(8.8);
    // delaymsg.set_zo(8.8);
    // delaymsg.set_wo(8.8);
    // delaymsg.set_xt(8.8);
    // delaymsg.set_yt(8.8);
    // delaymsg.set_zt(8.8);
    // delaymsg.set_wt(8.8);
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

    ros::Subscriber statusSub = nh.subscribe(uavName+"/delay/status",10,&status_callback);

    delayMessage::DelayMsg delaymsg;
    ros::Rate loop_rate(100);
    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    zmq_close(client);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;

}
