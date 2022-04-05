#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <zmq.h>
#include "../include/distributed_communication/delay.pb.h"
#include "../include/distributed_communication/config.h"
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

    int hwm = 1;
    if(0==zmq_connect(subscriber, GCS_command_sub_addr)){
            ROS_INFO("%s bind success", uavName.c_str());//return 0 if success
        }
        else{
            perror("server bind failed:");
            return -1;
        }
    zmq_setsockopt(subscriber, ZMQ_SUBSCRIBE, "", 0);
    zmq_setsockopt(subscriber, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(subscriber, ZMQ_RCVHWM, &hwm, sizeof(hwm));

    ros::Rate loop_rate(100);

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
            delaymsg.ParseFromString(str);

            if (delaymsg.is_from_keyboard())
            {
                ROS_INFO("%s subscribe: %s", uavName.c_str(), delaymsg.cmd().c_str());
                std_msgs::String msg;
                msg.data = delaymsg.cmd();
                cmd_pub.publish(msg);
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