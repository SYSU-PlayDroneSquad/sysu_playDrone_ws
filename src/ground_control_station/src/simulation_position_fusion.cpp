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

void uav_group_CB1(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                  const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                  const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                  const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                  const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};

    for(int i(0); i < 8; i++){
        pos_arr.x[i] = raw_pos_arr8[i]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[i]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[i]->pose.position.z;
    }
}

void uav_group_CB2(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                   const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                   const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                   const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                   const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};

    for(int i(8), j(0); i < 16; i++){
        pos_arr.x[i] = raw_pos_arr8[j]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[j]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[j]->pose.position.z;
        j++;
    }
}

void uav_group_CB3(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                   const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                   const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                   const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                   const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};
    for(int i(16), j(0); i < 24; i++){
        pos_arr.x[i] = raw_pos_arr8[j]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[j]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[j]->pose.position.z;
        j++;
    }
    cout << pos_arr << endl << endl;
}
void uav_group_CB4(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                   const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                   const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                   const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                   const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};
    for(int i(24), j(0); i < 32; i++){
        pos_arr.x[i] = raw_pos_arr8[j]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[j]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[j]->pose.position.z;
        j++;
    }
}
void uav_group_CB5(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                   const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                   const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                   const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                   const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};
    for(int i(32), j(0); i < 40; i++){
        pos_arr.x[i] = raw_pos_arr8[j]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[j]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[j]->pose.position.z;
        j++;
    }
}
void uav_group_CB6(const visualization_msgs::MarkerConstPtr &pos1, const visualization_msgs::MarkerConstPtr &pos2,
                   const visualization_msgs::MarkerConstPtr &pos3, const visualization_msgs::MarkerConstPtr &pos4,
                   const visualization_msgs::MarkerConstPtr &pos5, const visualization_msgs::MarkerConstPtr &pos6,
                   const visualization_msgs::MarkerConstPtr &pos7, const visualization_msgs::MarkerConstPtr &pos8,
                   const ros::Publisher& pub){
    vector<visualization_msgs::MarkerConstPtr> raw_pos_arr8{pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8};
    for(int i(40), j(0); i < 48; i++){
        pos_arr.x[i] = raw_pos_arr8[j]->pose.position.x;
        pos_arr.y[i] = raw_pos_arr8[j]->pose.position.y;
        pos_arr.z[i] = raw_pos_arr8[j]->pose.position.z;
        j++;
    }
}

void pub_pos_arr_CB(const ros::TimerEvent &Event, ros::Publisher &pub_pos_arr){
    pub_pos_arr.publish(pos_arr);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "position_fusion");
    ros::NodeHandle nh;

    // 需要用message_filter容器对多个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav1(nh, "/uav1/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav2(nh, "/uav2/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav3(nh, "/uav3/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav4(nh, "/uav4/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav5(nh, "/uav5/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav6(nh, "/uav6/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav7(nh, "/uav7/command/pose_marker", 10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav8(nh, "/uav8/command/pose_marker", 10);

    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav9(nh,"/uav9/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav10(nh,"/uav10/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav11(nh,"/uav11/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav12(nh,"/uav12/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav13(nh,"/uav13/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav14(nh,"/uav14/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav15(nh,"/uav15/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav16(nh,"/uav16/command/pose_marker",10);

    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav17(nh,"/uav17/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav18(nh,"/uav18/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav19(nh,"/uav19/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav20(nh,"/uav20/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav21(nh,"/uav21/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav22(nh,"/uav22/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav23(nh,"/uav23/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav24(nh,"/uav24/command/pose_marker",10);

    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav25(nh,"/uav25/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav26(nh,"/uav26/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav27(nh,"/uav27/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav28(nh,"/uav28/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav29(nh,"/uav29/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav30(nh,"/uav30/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav31(nh,"/uav31/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav32(nh,"/uav32/command/pose_marker",10);

    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav33(nh,"/uav33/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav34(nh,"/uav34/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav35(nh,"/uav35/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav36(nh,"/uav36/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav37(nh,"/uav37/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav38(nh,"/uav38/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav39(nh,"/uav39/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav40(nh,"/uav40/command/pose_marker",10);

    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav41(nh,"/uav41/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav42(nh,"/uav42/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav43(nh,"/uav43/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav44(nh,"/uav44/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav45(nh,"/uav45/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav46(nh,"/uav46/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav47(nh,"/uav47/command/pose_marker",10);
    message_filters::Subscriber<visualization_msgs::Marker> sub_pos_uav48(nh,"/uav48/command/pose_marker",10);

    // 将多个话题的数据进行同步
    typedef message_filters::sync_policies::ApproximateTime<
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker,
            visualization_msgs::Marker, visualization_msgs::Marker> syncPolicy;
    message_filters::Synchronizer<syncPolicy> uavGroup1(syncPolicy(10),
            sub_pos_uav1, sub_pos_uav2, sub_pos_uav3, sub_pos_uav4,
            sub_pos_uav5, sub_pos_uav6, sub_pos_uav7, sub_pos_uav8);
    message_filters::Synchronizer<syncPolicy> uavGroup2(syncPolicy(10),
            sub_pos_uav9, sub_pos_uav10, sub_pos_uav11,  sub_pos_uav12,
            sub_pos_uav13, sub_pos_uav14, sub_pos_uav15, sub_pos_uav16);
    message_filters::Synchronizer<syncPolicy> uavGroup3(syncPolicy(10),
            sub_pos_uav17, sub_pos_uav18, sub_pos_uav19, sub_pos_uav20,
            sub_pos_uav21, sub_pos_uav22, sub_pos_uav23, sub_pos_uav24);
    message_filters::Synchronizer<syncPolicy> uavGroup4(syncPolicy(10),
            sub_pos_uav25, sub_pos_uav26, sub_pos_uav27, sub_pos_uav28,
            sub_pos_uav29, sub_pos_uav30, sub_pos_uav31, sub_pos_uav32);
    message_filters::Synchronizer<syncPolicy> uavGroup5(syncPolicy(10),
            sub_pos_uav33, sub_pos_uav34, sub_pos_uav35, sub_pos_uav36,
            sub_pos_uav37, sub_pos_uav38, sub_pos_uav39, sub_pos_uav40);
    message_filters::Synchronizer<syncPolicy> uavGroup6(syncPolicy(10),
            sub_pos_uav41, sub_pos_uav42, sub_pos_uav43, sub_pos_uav44,
            sub_pos_uav45, sub_pos_uav46, sub_pos_uav47, sub_pos_uav48);

    // publisher
    ros::Publisher pub_pos_arr = nh.advertise<ground_control_station::Array3>("pos_arr", 100);

    // 指定一个回调函数，就可以实现多个话题数据的同步获取
    uavGroup1.registerCallback(boost::bind(&uav_group_CB1, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));
    uavGroup2.registerCallback(boost::bind(&uav_group_CB2, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));
    uavGroup3.registerCallback(boost::bind(&uav_group_CB3, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));
    uavGroup4.registerCallback(boost::bind(&uav_group_CB4, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));
    uavGroup5.registerCallback(boost::bind(&uav_group_CB5, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));
    uavGroup6.registerCallback(boost::bind(&uav_group_CB6, _1, _2, _3, _4, _5, _6, _7, _8, pub_pos_arr));

    int uavNum = 48;
    pos_arr.x.resize(uavNum);
    pos_arr.y.resize(uavNum);
    pos_arr.z.resize(uavNum);

    ros::MultiThreadedSpinner spinner(8);
    ros::Timer pub_timer = nh.createTimer(ros::Duration(0.02), boost::bind(pub_pos_arr_CB, _1, pub_pos_arr));
    cout << "start spin" << endl;
    ros::spin();
    return 0;
}
