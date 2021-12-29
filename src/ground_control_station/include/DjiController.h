#ifndef HECTOR_QUADROTOR_WS_DJICONTROLLER_H
#define HECTOR_QUADROTOR_WS_DJICONTROLLER_H

// ROS includes
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/tf.h>

// DJI SDK includes
#include "dji_sdk/dji_sdk.h"             // 飞行姿态、模式
#include "dji_sdk/DroneTaskControl.h"    // 起飞降落返航
#include "dji_sdk/DroneArmControl.h"     // 解锁电机
#include <dji_sdk/SetLocalPosRef.h>      // GPS相关
#include <dji_sdk/QueryDroneVersion.h>   // 飞控版本获取
#include <dji_sdk/SDKControlAuthority.h> // 获取板载控制权

// Others includes
#include <cmath>
#include <utility>
#include "ground_control_station/Array3.h"

using std::string;
using std::cout;
using std::endl;

class DjiN3Controller {
public:
    DjiN3Controller()= default;
    DjiN3Controller(ros::NodeHandle nh, string uavName) {
        // 句柄
        _nh = nh;
        _uavName = std::move(uavName);
        // 控制参考坐标系
        _frame = true; // true = ENU ; false = FLU
        // 速度消息
        _xCmd = _yCmd = _zCmd = _yawCmd = 0;
        _setVelYaw_msg.axes.resize(4);

        // 姿态角
        _yaw = _roll = _pitch = 0.0;
        // 速度限制
        _vel_limt = 1.0;
        // 速度步进值
        _step = 1.0;
        //　无人机任务　client
        _drone_task_client = nh.serviceClient<dji_sdk::DroneTaskControl>(
                _uavName + "/dji_sdk/drone_task_control");
        // 解锁电机
        _motor_control_client = nh.serviceClient<dji_sdk::DroneArmControl>(
                _uavName + "/dji_sdk/drone_arm_control");

        // 速度订阅与发布

        _setVelYaw_pub = nh.advertise<sensor_msgs::Joy>(
                _uavName + "/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
        _vel_list_sub = nh.subscribe<ground_control_station::Array3>(
                "/vel_list", 100,
                boost::bind(&DjiN3Controller::vel_list_sub_CB, this, _1, _uavName, _setVelYaw_pub));

        // FLU 到 ENU 的旋转四元数订阅
        _dji_att_sub = _nh.subscribe<geometry_msgs::QuaternionStamped>(
                _uavName +"/dji_sdk/attitude", 10,
                boost::bind(&DjiN3Controller::attitude_CB, this, _1));

        /*
        // 飞行状态与模式
        _flight_status = 255;
        _display_mode = 255;

         // 位置消息
        _setPosYaw_msg.axes.emplace_back(_xCmd);
        _setPosYaw_msg.axes.emplace_back(_yCmd);
        _setPosYaw_msg.axes.emplace_back(_zCmd);
        _setPosYaw_msg.axes.emplace_back(_yawCmd);

        // 位置 publisher
        _setPosYaw_pub = nh.advertise<sensor_msgs::Joy>(
                _uavName + "/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
        // 状态
        _status_pub = _nh.advertise<std_msgs::String>(_uavName + "/status", 10);

        // gps health
        _gps_health_sub = _nh.subscribe(_uavName + "/dji_sdk/gps_health", 10, &DjiN3Controller::gps_health_CB, this);

        // 飞行状态与模式订阅器
        _flightStatusSub = nh.subscribe(
                _uavName + "/dji_sdk/flight_status", 10, &DjiN3Controller::flight_status_CB, this);
        _displayModeSub = nh.subscribe(
                _uavName + "/dji_sdk/display_mode", 10, &DjiN3Controller::display_mode_CB, this);
        */
    }

    ~DjiN3Controller() = default;

    /// ========================== 基本控制 ==========================
    void takeoff() {
        ros::Time begin = ros::Time::now();
        _setVelYaw_msg.axes[0] = 0;
        _setVelYaw_msg.axes[1] = 0;
        _setVelYaw_msg.axes[2] = 1;
        _setVelYaw_msg.axes[3] = 0;
        ros::Rate rate(100);
        while(ros::Time::now() - ros::Duration(5) < begin){
            _setVelYaw_pub.publish(_setVelYaw_msg);
            rate.sleep();
        }
        cout << _uavName << ": Takeoff success!" << endl;
    }

    void land() {
        dji_sdk::DroneTaskControl droneTaskControl;
        droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
        _drone_task_client.call(droneTaskControl);
        if (droneTaskControl.response.result) {
            cout << _uavName << ": Land success!" << endl;
        } else {
            cout << "\033[31mError: \033[0m" << "land failed" << endl;
        }
    }

    void stop() {
        _xCmd = 0;
        _yCmd = 0;
        _zCmd = 0;
        _yawCmd = 0;
        _setVelYaw_msg.axes[0] = _xCmd;
        _setVelYaw_msg.axes[1] = _yCmd;
        _setVelYaw_msg.axes[2] = _zCmd;
        _setVelYaw_msg.axes[3] = _yawCmd;
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the stop command!" << endl;
    }

    void clear(){
        _xCmd = 0;
        _yCmd = 0;
        _zCmd = 0;
        _yawCmd = 0;
        _setVelYaw_msg.axes[0] = _xCmd;
        _setVelYaw_msg.axes[1] = _yCmd;
        _setVelYaw_msg.axes[2] = _zCmd;
        _setVelYaw_msg.axes[3] = _yawCmd;
    }

    void forward() {
        clear();
        if (_xCmd >= 0 && _xCmd < _vel_limt) {
            _xCmd += _step;
        } else if (_xCmd < 0) {
            _xCmd = 0;
        } else {
            _xCmd = _vel_limt;
        }
        get_vel();
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the forward command!" << endl;
    }

    void backward() {
        clear();
        if (_xCmd <= 0 && _xCmd > -_vel_limt) {
            _xCmd -= _step;
        } else if (_xCmd > 0) {
            _xCmd = 0;
        } else {
            _xCmd = -_vel_limt;
        }
        get_vel();
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the backward command!" << endl;
    }

    void turn_right() {
        clear();
        if (_yCmd <= 0 && _yCmd > -_vel_limt) {
            _yCmd -= _step;
        } else if (_yCmd > 0) {
            _yCmd = 0;
        } else {
            _yCmd = -_vel_limt;
        }
        get_vel();
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the right turn command!" << endl;
    }

    void turn_left() {
        clear();
        if (_yCmd >= 0 && _yCmd < _vel_limt) {
            _yCmd += _step;
        } else if (_yCmd < 0) {
            _yCmd = 0;
        } else {
            _yCmd = _vel_limt;
        }
        get_vel();
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the left turn command!" << endl;
    }

    void upward() {
        clear();
        if (_zCmd >= 0 && _zCmd < _vel_limt) {
            _zCmd += _step;
        } else if (_zCmd < 0) {
            _zCmd = 0;
        } else {
            _zCmd = _vel_limt;
        }
        _setVelYaw_msg.axes[2] = _zCmd;
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the upward command!" << endl;
    }

    void down() {
        clear();
        if (_zCmd <= 0 && _zCmd > -_vel_limt) {
            _zCmd -= _step;
        } else if (_zCmd > 0) {
            _zCmd = 0;
        } else {
            _zCmd = -_vel_limt;
        }
        _setVelYaw_msg.axes[2] = _zCmd;
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the right turn command!" << endl;
    }

    void rotate_left() {
        clear();
        if (_yawCmd >= 0 && _zCmd < _vel_limt) {
            _yawCmd += _step;
        } else if (_yawCmd < 0) {
            _yawCmd = 0;
        } else {
            _yawCmd = _vel_limt;
        }
        _setVelYaw_msg.axes[3] = _yawCmd;
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the left rotate command!" << endl;
    }

    void rotate_right() {
        clear();
        if (_yawCmd <= 0 && _yawCmd > -_vel_limt) {
            _yawCmd -= _step;
        } else if (_yawCmd > 0) {
            _yawCmd = 0;
        } else {
            _yawCmd = -_vel_limt;
        }
        _setVelYaw_msg.axes[3] = _yawCmd;
        _setVelYaw_pub.publish(_setVelYaw_msg);
        cout << _uavName << ": Successfully sent the right turn command!" << endl;
    }
    // 机身(FLU)&大地(ENU)坐标系切换
    bool _frame{};
    void switch_frame(){
        _frame = !_frame;
        if(_frame){
            printf("\33[33m%s\33[0m: \33[32mframe\33[0m = \33[35mFLU\33[0m\n\n", _uavName.c_str());
        } else {
            printf("\33[33m%s\33[0m: \33[32mframe\33[0m = \33[34mENU\33[0m\n\n", _uavName.c_str());
        }
    }

    void round_up(){
        _round_up = !_round_up;
        if(_round_up){
            cout << "start round up!" << endl;
        } else{
            cout << "end round up" << endl;
        }
    }

    void go_home(){}

    /*
    void set_pos(sensor_msgs::Joy pos){///待完善-不可用
        _xCmd = pos.axes[0];
        _yCmd = pos.axes[1];
        _zCmd = pos.axes[2];
        _yawCmd = pos.axes[3];
        pos_enu2flu(_setPosYaw_msg);
        cout << "\033[33m_setPosYaw_msg:\033[0m\n" << _setPosYaw_msg << endl;
        _setPosYaw_pub.publish(_setPosYaw_msg);
    }

    void set_vel(sensor_msgs::Joy vel){///待完善-不可用
        _xCmd = vel.axes[0];
        _yCmd = vel.axes[1];
        _zCmd = vel.axes[2];
        _yawCmd = vel.axes[3];
        if(_frame){
            vel_enu2flu();
        } else {
            _setVelYaw_msg.axes[0] = _xCmd;
            _setVelYaw_msg.axes[1] = _yCmd;
        }
        _setVelYaw_pub.publish(vel);
    }

     // 设置本地坐标
    void set_local_position() {
        ros::ServiceClient set_local_pos_reference;
        set_local_pos_reference = _nh.serviceClient<dji_sdk::SetLocalPosRef>(
                _uavName + "/dji_sdk/set_local_pos_ref");
        dji_sdk::SetLocalPosRef localPosReferenceSetter;
        set_local_pos_reference.call(localPosReferenceSetter);
        ros::Rate r(10);
        int count = 0;
        while(ros::ok() && count < 10){
            if (!localPosReferenceSetter.response.result) {
                cout << "\033[31mError: \033[0m"<< _uavName <<
                     " GPS health insufficient,failed to set the local reference frame, set again." << endl;
            } else {
                cout << "GPS health,local reference frame set succeed!" << endl;
                _local_frame = string("1");
                break;
            }
            count++;
            r.sleep();
        }
        if(_local_frame != "1"){
            cout << "\033[31mError: \033[0m"<< _uavName <<
                 " GPS health insufficient,failed to set the local reference frame." << endl;
        }
    }
    */

    /// ========================== 飞控 ==========================
    // 获取板载控制权
    bool obtain_control() {
        dji_sdk::SDKControlAuthority authority;
        authority.request.control_enable = 1;
        ros::ServiceClient sdk_ctrl_authority_service;
        sdk_ctrl_authority_service = _nh.serviceClient<dji_sdk::SDKControlAuthority>(
                _uavName + "/dji_sdk/sdk_control_authority");
        sdk_ctrl_authority_service.call(authority);

        if (!authority.response.result) {
            cout << "\033[31mError: \033[0m"<< _uavName << " Obtain control failed!" << endl;
            return false;
        } else {
            cout << "Obtain control success!" << endl;
        }

        return true;
    }

    // 获取无人机飞控型号
    void get_version() {
        ros::ServiceClient query_version_service;
        query_version_service = _nh.serviceClient<dji_sdk::QueryDroneVersion>(
                _uavName + "/dji_sdk/query_drone_version");
        dji_sdk::QueryDroneVersion query;
        query_version_service.call(query);
        if (query.response.hardware == "N3") {
            cout << "\033[33mUav version: \033[0m" << query.response.hardware << endl;
        }
    }

    // 电机
    void motor_control() {
        dji_sdk::DroneArmControl enable;
        enable.request.arm = 1;
        _motor_control_client.call(enable);
        if (enable.response.result) {
            cout << "enable motor success" << endl;
        } else {
            cout << "enable motor failed" << endl;
        }
    }
    /*
    // 判断遥控器是否连接
    void spinning_motor() {
        ros::Time start_time = ros::Time::now();
        int flight = _flight_status;
        int display = _display_mode;
        std::cout << "flight_1 = " << flight << std::endl;
        std::cout << "display_1 = " << display << std::endl << std::endl;

        while (_flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
               _display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
               ros::Time::now() - start_time < ros::Duration(5)) {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
        if (ros::Time::now() - start_time > ros::Duration(5)) {
            cout << "\033[31mError:\033[0m"<< _uavName << "Takeoff failed. Motors are not spinnning." << endl;
        } else {
            start_time = ros::Time::now();
            ROS_INFO("Motor Spinning ...");
            ros::spinOnce();
        }
    }
    */


    /// ========================== 回调 ==========================
    // FLU 到 ENU 的旋转四元数订阅
    void attitude_CB(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
        quat2Tf2Rpy(msg);
    }

    void vel_list_sub_CB(const ground_control_station::Array3::ConstPtr &vel_list, string uavName, ros::Publisher &pub) {
        if(_round_up){
            cout << "round up" << endl;
            int id = _uavName[3] - '0' - 1;
            _xCmd = vel_list->x[0];
            _yCmd = vel_list->y[0];
            _zCmd = vel_list->z[0];
            get_vel();
            pub.publish(_setVelYaw_msg);
        }
    }

    /*
    // 飞行状态订阅回调
    void flight_status_CB(const std_msgs::UInt8::ConstPtr &msg) {
        _flight_status = msg->data;
    }

    // 显示模式订阅回调
    void display_mode_CB(const std_msgs::UInt8::ConstPtr &msg) {
        _display_mode = msg->data;
    }

    // gps 信号等级订阅
    void gps_health_CB(const std_msgs::UInt8::ConstPtr &msg){
        _gps_health = msg->data;
    }
    */

    /// ========================== 辅助函数 ==========================
    // 四元数转rpy
    void quat2Tf2Rpy(const geometry_msgs::QuaternionStamped::ConstPtr &msg){
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->quaternion,quat);
        tf::Matrix3x3(quat).getRPY(_roll, _pitch, _yaw);
        _yawRate = _yaw * 180 / M_PI;
    }

    // 速度、位置 ENU 平面坐标系转 FLU 平面坐标系
    void vel_enu2flu(){//速度控制转换
        ros::spinOnce();
        double xVel, yVel;
        // 二维坐标系旋转变换
        xVel = _xCmd * cos(-_yaw) + _yCmd * sin(-_yaw);
        yVel = _yCmd * cos(-_yaw) - _xCmd * sin(-_yaw);
        // 大地坐标系的速度
        _setVelYaw_msg.header.stamp = ros::Time::now();
        _setVelYaw_msg.header.frame_id = "FLU";
        _setVelYaw_msg.axes[0] = xVel;
        _setVelYaw_msg.axes[1] = yVel;
        _setVelYaw_msg.axes[2] = _zCmd;
    }

    void pos_enu2flu(sensor_msgs::Joy &msg){///位置控制转换 待完善-不可用
        ros::spinOnce();
        double x, y;
        // 二维坐标系旋转变换
        x = _xCmd * cos(-_yaw) + _yCmd * sin(-_yaw);
        y = _yCmd * cos(-_yaw) - _xCmd * sin(-_yaw);

        msg.axes[0] = x;
        msg.axes[1] = y;
        msg.axes[2] = _zCmd;
        msg.axes[3] = _rate;
    }

    void get_yawrate(){
        _rate = -_yawRate;
        cout << "get rate sucess,rate = " << _rate << endl;
    }

    /*!
      * 状态信息
      * 例子：uav1#105
      *      消息头：auv1
      *      第一位：通信，1 或 0
      *      第二位：本地原点设置、1 或 0
      *      第三位：GPS 信号等级 0 ～ 5
      *
      */
    /*
    void pub_status(){
        // string communication = "1";
        std_msgs::String status;
        // status.data = _uavName +"#" + communication + _local_frame + std::to_string(_gps_health);
        status.data = _uavName + "#" + std::to_string(_gps_health);
        _status_pub.publish(status);
    }
    */

    // 赋值速度消息
    void get_vel(){
        if(_frame){
            vel_enu2flu();
        } else {
            _setVelYaw_msg.header.stamp = ros::Time::now();
            _setVelYaw_msg.header.frame_id = "ENU";
            _setVelYaw_msg.axes[0] = _xCmd;
            _setVelYaw_msg.axes[1] = _yCmd;
            _setVelYaw_msg.axes[2] = _zCmd;
        }
    }


private:
    // ros 句柄
    ros::NodeHandle _nh;
    // 无人机名
    string _uavName;
    // 飞行任务 client
    ros::ServiceClient _drone_task_client;
    // 解锁电机
    ros::ServiceClient _motor_control_client;
    // 速度
    static sensor_msgs::Joy _setVelYaw_msg;
    ros::Publisher _setVelYaw_pub;
    ros::Subscriber _vel_list_sub;

    // 速度消息辅助参数
    float _xCmd, _yCmd, _zCmd, _yawCmd, _vel_limt, _step;
    // FLU 机身坐标系到 ENU 地面坐标系的旋转四元数
    ros::Subscriber _dji_att_sub;
    // FLU 到 ENU 的姿态角变化
    static double _roll, _pitch, _yaw, _yawRate, _rate;

    // 围捕
    static bool _round_up;

    /*
    // 位置
    sensor_msgs::Joy _setPosYaw_msg;
    ros::Publisher _setPosYaw_pub;

    // 飞行状态与模式参数
    uint8_t _flight_status{};
    uint8_t _display_mode{};

    // 飞行状态与模式订阅器
    ros::Subscriber _flightStatusSub;
    ros::Subscriber _displayModeSub;

    // 状态
    ros::Publisher _status_pub;
    static string _local_frame;
    // GPS
    ros::Subscriber _gps_health_sub;
    static uint8_t _gps_health;
    */
};
// string DjiN3Controller::_local_frame = string("0");
// uint8_t DjiN3Controller::_gps_health = 0;
sensor_msgs::Joy DjiN3Controller::_setVelYaw_msg;
bool DjiN3Controller::_round_up = false;
double DjiN3Controller::_roll = 0.0;
double DjiN3Controller::_pitch = 0.0;
double DjiN3Controller::_yaw = 0.0;
double DjiN3Controller::_yawRate = 0.0;
double DjiN3Controller::_rate = 0.0;



#endif //HECTOR_QUADROTOR_WS_DJICONTROLLER_H
