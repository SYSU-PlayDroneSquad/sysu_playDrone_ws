// ros include
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
// other include
#include <boost/thread/thread.hpp>
#include "ground_control_station/Array3.h"

using namespace message_filters;
using namespace std;

ground_control_station::Array3 pos_arr;

void imu_callback(const visualization_msgs::MarkerConstPtr &pos1,
                  const visualization_msgs::MarkerConstPtr &pos2,
                  const visualization_msgs::MarkerConstPtr &pos3,
                  const visualization_msgs::MarkerConstPtr &pos4,
                  const visualization_msgs::MarkerConstPtr &pos5,
                  const visualization_msgs::MarkerConstPtr &pos6,
                  const visualization_msgs::MarkerConstPtr &pos7,
                  const visualization_msgs::MarkerConstPtr &pos8,
                  const ros::Publisher& pub)
{
    cout << pos2->pose.position << endl;
    pos_arr.x[0] = pos1->pose.position.x;
    pos_arr.x[1] = pos2->pose.position.x;
    pos_arr.x[2] = pos3->pose.position.x;
    pos_arr.x[3] = pos4->pose.position.x;
    pos_arr.x[4] = pos5->pose.position.x;
    pos_arr.x[5] = pos6->pose.position.x;
    pos_arr.x[6] = pos7->pose.position.x;
    pos_arr.x[7] = pos8->pose.position.x;

    pos_arr.y[0] = pos1->pose.position.y;
    pos_arr.y[1] = pos2->pose.position.y;
    pos_arr.y[2] = pos3->pose.position.y;
    pos_arr.y[3] = pos4->pose.position.y;
    pos_arr.y[4] = pos5->pose.position.y;
    pos_arr.y[5] = pos6->pose.position.y;
    pos_arr.y[6] = pos7->pose.position.y;
    pos_arr.y[7] = pos8->pose.position.y;

    pos_arr.z[0] = pos1->pose.position.z;
    pos_arr.z[1] = pos2->pose.position.z;
    pos_arr.z[2] = pos3->pose.position.z;
    pos_arr.z[3] = pos4->pose.position.z;
    pos_arr.z[4] = pos5->pose.position.z;
    pos_arr.z[5] = pos6->pose.position.z;
    pos_arr.z[6] = pos7->pose.position.z;
    pos_arr.z[7] = pos8->pose.position.z;


    cout << pos_arr << endl;
    pub.publish(pos_arr);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_fusion");
    ros::NodeHandle nh;

    // publisher
    ros::Publisher pub_pos_arr = nh.advertise<ground_control_station::Array3>("pos_arr", 100);

    // 需要用message_filter容器对多个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav1(nh,"/uav1/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav2(nh,"/uav2/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav3(nh,"/uav3/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav4(nh,"/uav4/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav5(nh,"/uav5/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav6(nh,"/uav6/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav7(nh,"/uav7/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav8(nh,"/uav8/command/pose_marker",10);

    // 将多个话题的数据进行同步
    typedef message_filters::sync_policies::ApproximateTime<
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker> syncPolicy;

    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10),
                                  sub_pos_uav1, sub_pos_uav2, sub_pos_uav3, sub_pos_uav4,
                                  sub_pos_uav5, sub_pos_uav6, sub_pos_uav7, sub_pos_uav8);

    // 指定一个回调函数，就可以实现多个话题数据的同步获取
    sync.registerCallback(boost::bind(&imu_callback, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));

    int uavNum = 8;
    pos_arr.x.resize(uavNum);
    pos_arr.y.resize(uavNum);
    pos_arr.z.resize(uavNum);


    cout << pos_arr << endl;

    ros::MultiThreadedSpinner spinner(8);
    ros::spin();
    return 0;
}
