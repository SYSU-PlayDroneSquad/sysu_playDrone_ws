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

#include "ground_control_station/Status.h"
#include "ground_control_station/StatusArray.h"

using namespace std;

void* ctx = zmq_ctx_new();
void* subscriber = zmq_socket(ctx, ZMQ_SUB);
vector<void*> subscriberList = vector<void*>(51, zmq_socket(ctx, ZMQ_SUB));
string uavName("uav");
int numUav;
int seq[51];

ros::Publisher attitudePub;
ros::Publisher positionPub;

delayMessage::DelayMsg delaymsg;
ground_control_station::Status status;
ros::Time stamp;

void assignStatus() {
    stamp.sec = delaymsg.send_time();
    // Assign header
    status.header.seq      = delaymsg.msg_id();
    status.header.stamp    = stamp;
    status.header.frame_id = delaymsg.str();

    // Assign GPS_health
    status.lv_gps          = delaymsg.gps();

    // Assign attitude
    status.quaternion.x    = delaymsg.x();
    status.quaternion.y    = delaymsg.y();
    status.quaternion.z    = delaymsg.z();
    status.quaternion.w    = delaymsg.w();

    // assign GPS_position
    status.latitude        = delaymsg.lat();
    status.longitude       = delaymsg.lon();
    status.altitude        = delaymsg.alt();

    status.seq = seq[delaymsg.uav_id()] + 1;
    seq[delaymsg.uav_id()] = status.seq;
    status.id = delaymsg.uav_id();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_node");
    ros::NodeHandle nh;
    ros::param::get("~uav_name", uavName);
    ros::param::get("~num_uav", numUav);

    ground_control_station::StatusArray statusArray;
    vector<ground_control_station::Status> statusarr(numUav);

    // publish msg to ground control station
    ros::Publisher statusPub = nh.advertise<ground_control_station::StatusArray>("status", 10);

   int hwm = 1;
    // 绑定地址
    for(int i=0;i<numUav;++i){
        if(0==zmq_connect(subscriberList[i], sub_ground_addr_list[i])){
            ROS_INFO("client %d bind success", i);//return 0 if success
        }
        else{
            perror("server bind failed:");
            return -1;
        }
        zmq_setsockopt(subscriberList[i], ZMQ_SUBSCRIBE, "", 0);
        zmq_setsockopt(subscriberList[i], ZMQ_SNDHWM, &hwm, sizeof(hwm));
        zmq_setsockopt(subscriberList[i], ZMQ_RCVHWM, &hwm, sizeof(hwm));
    }

    ros::Rate loop_rate(50);
    // zmq_msg_t recv_msg;
    // zmq_msg_init(&recv_msg);

    while(ros::ok()){
        ros::Time start_time = ros::Time::now();
        std_msgs::String abc;
         while (ros::Time::now() - start_time < ros::Duration(0.006)){
            delaymsg.Clear();
            zmq_msg_t recv_msg;
            zmq_msg_init(&recv_msg);
            for (int i = 0; i < numUav; i++)
            {
  //            delaymsg.Clear();
  //	          zmq_msg_t recv_msg;
  //            zmq_msg_init(&recv_msg);
                int recv_byte = zmq_msg_recv(&recv_msg,subscriberList[i],0);//ZMQ_DONTWAIT

                if(recv_byte > 0)
                {
                  ROS_INFO("Ground control station receive message (%d bytes) success.", recv_byte);

                  string str;
                  // std_msgs::String status;h
                  str.assign((char*)zmq_msg_data(&recv_msg),recv_byte);
                  zmq_msg_close(&recv_msg);
                  //    int index = str.find_first_of('\0');
                  //    ROS_INFO("%d",index);
                  //    delaymsg.ParseFromString(str.substr(0,index));
                  delaymsg.ParseFromString(str);
                  assignStatus();
//
                  ROS_INFO("The status of uav%d : %d", delaymsg.uav_id(), status.lv_gps);
                  ROS_INFO("The sequence: %d, the stamp: %f, the frame_id: %s", status.seq, delaymsg.send_time(), delaymsg.str().c_str());
                  ROS_INFO("The latitude: %f, the longitude: %f, the altitude: %f", status.latitude, status.longitude, status.altitude);
                  ROS_INFO("The x: %f, the y: %f, the z: %f, the w: %f", status.quaternion.x, status.quaternion.y, status.quaternion.z, status.quaternion.w);

                  statusarr[status.id - 1] = status;

                }
            }
        }
        statusArray.StatusArray = statusarr;
        statusPub.publish(statusArray);

        // std::cout << ros::Time::now() - start_time << std::endl;
        // start_time = ros::Time::now();
        // ros::spinOnce();
        // loop_rate.sleep();
    }


    zmq_close(subscriber);
    zmq_ctx_destroy(ctx);

    // ros::shutdown();
    return 0;
}
