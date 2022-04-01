// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>

// customer include
#include "DjiController.h"
#include "CommandParser.h"

using std::cout;
using std::endl;
using std::string;

// dji 键盘控制
void keyCallBack(const std_msgs::String::ConstPtr cmd, CommandParser &command_parser ) {
    string data = cmd->data;
    command_parser.check_data(data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_parser");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // 无人机的名字
    string uavName;
    // 从参数服务器获取 uavName
    nh.getParam("/command_parser/uavName", uavName);
    // 显示无人机名字
    printf("\33[33mstation\33[0m: \33[32muavName\33[0m = \33[36m%s\33[0m\n\n", uavName.c_str());

    // 实例化地面站指令解释器
    CommandParser command_parser(n, uavName);

    // dji_osdk 初始化
    cout << "\033[33m初始化开始...\033[0m" << endl;
    command_parser.osdk_init();

    // 键盘控制
    ros::Subscriber key_subscriber = nh.subscribe<std_msgs::String>(
            "key", 1, boost::bind(keyCallBack, _1, command_parser));

    // 状态发布
    //ros::Timer status_pub_timer = nh.createTimer(ros::Duration(1), &CommandParser::timerCallBack, &command_parser);

    ros::spin();

}



