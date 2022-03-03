#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <zmq.h>
#include "../include/cloud/delay.pb.h"
//#include "../include/delay/delay.pb.cc"
#include "../include/cloud/config.h"
#include <std_msgs/String.h>
using namespace std;

void* ctx = zmq_ctx_new();
void* subscriber = zmq_socket(ctx, ZMQ_SUB);
string uavName("uav");

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_node");
    ros::NodeHandle nh;
    ros::param::get("~uav_name", uavName);
    delayMessage::DelayMsg delaymsg;

    ros::Publisher cmd_pub = nh.advertise<std_msgs::String>("key", 1000);

    if(0 == zmq_connect(subscriber, subaddr)){
        ROS_INFO("%s client connect success", uavName.c_str());//return 0 if success
    }else{
        perror(string(uavName+"client connect failed:").c_str());
        return -1;
    }
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        delaymsg.Clear();
        zmq_msg_t recv_msg;
        zmq_msg_init(&recv_msg);
        int recv_byte = zmq_msg_recv(&recv_msg,subscriber,0);//ZMQ_DONTWAIT

        if(recv_byte > 0)
        {
            ROS_INFO("%s receive message (%d bytes) success.",uavName.c_str(), recv_byte);

            string str;
            str.assign((char*)zmq_msg_data(&recv_msg),recv_byte);
            zmq_msg_close(&recv_msg);
            //    int index = str.find_first_of('\0');
            //    ROS_INFO("%d",index);
            //    delaymsg.ParseFromString(str.substr(0,index));
            delaymsg.ParseFromString(str);
            if (delaymsg.uav_id() == 0)
            {
                ROS_INFO("%s subscribe: %s", uavName.c_str(), delaymsg.cmd().c_str());
                std_msgs::String msg;
                msg.data = delaymsg.cmd();
                cmd_pub.publish(msg);
            }
            double durationTime=ros::WallTime::now().toSec()-delaymsg.send_time();
            ROS_INFO("%s subscribe: msg_id:%d send time:%.9lf",uavName.c_str(),delaymsg.msg_id(),durationTime);
            ROS_INFO("%s subscribe: id:%d, lat:%f, lon:%f, alt:%f, vx:%f, vy:%f, vz:%f",uavName.c_str(), delaymsg.uav_id(), delaymsg.lat(), delaymsg.lon(), delaymsg.alt(), delaymsg.vx(), delaymsg.vy(), delaymsg.vz());
        }    

        ros::spinOnce();
        loop_rate.sleep();
    }


    zmq_close(subscriber);
    zmq_ctx_destroy(ctx);

    ros::shutdown();
    return 0;
}
