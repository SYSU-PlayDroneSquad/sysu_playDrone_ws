// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// customer include
#include "HectorQuadrotor.h"
#include "UavGroup.h"

using std::cout;
using std::endl;
using std::string;


// gazebo 键盘控制
void ctrl_keyCB(const std_msgs::String::ConstPtr cmd, UavGroup<HectorQuadrotor> &controller) {
    string data = cmd->data;
    controller.baseControl(data);
}
// gazebo 位置控制
void set_group_position_CB(const sensor_msgs::Joy::ConstPtr msg, UavGroup<HectorQuadrotor> &controller){// 待完善
    controller.pos_globalControl(*msg);
}
// gazebo 速度控制
void set_group_velocity_CB(const sensor_msgs::Joy::ConstPtr msg, UavGroup<HectorQuadrotor> &controller){// 待完善
    controller.vel_globalControl(*msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_control_station");
    ros::NodeHandle nh;
    ros::NodeHandle n;
    // 要控制的无人机数量
    int uavNumbers;

    // dji 接口或 gazebo 仿真接口
    string dji_gazebo;

    // 从参数服务器获取 uavNumbers 和 dji_gazebo 变量值
    nh.param("/ground_control_station/uavNumbers", uavNumbers, 10);


    printf("\33[33mstation\33[0m: \33[32muavNumbers\33[0m = \33[36m%d\33[0m\n\n", uavNumbers);

    // 创建 gazebo 无人机类型组
    UavGroup<HectorQuadrotor> hec_uav_group(n, uavNumbers);

    // 键盘控制无人机组
    ros::Subscriber key_subscriber2 = nh.subscribe<std_msgs::String>(
            "key", 1,boost::bind(ctrl_keyCB, _1, hec_uav_group));

    // 位置控制
    ros::Subscriber set_group_position_sub = nh.subscribe<sensor_msgs::Joy>(
            "set_group_position", 1, boost::bind(set_group_position_CB, _1, hec_uav_group));

    // 速度控制
    ros::Subscriber set_group_velocity_sub = nh.subscribe<sensor_msgs::Joy>(
            "set_group_velocity", 1, boost::bind(set_group_velocity_CB, _1, hec_uav_group));

    ros::spin();


}



