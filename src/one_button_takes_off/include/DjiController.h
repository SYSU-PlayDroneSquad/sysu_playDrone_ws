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
#include <dji_sdk/MFIOConfig.h>          // PWM

// Others includes
#include <cmath>
#include <utility>
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
        _attack_client = nh.serviceClient<dji_sdk::MFIOConfig>(_uavName + "/dji_sdk/mfio_config");

        // 速度订阅与发布

        _setVelYaw_pub = nh.advertise<sensor_msgs::Joy>(
                _uavName + "/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);

        // FLU 到 ENU 的旋转四元数订阅
        _dji_att_sub = _nh.subscribe<geometry_msgs::QuaternionStamped>(
                _uavName +"/dji_sdk/attitude", 10,
                boost::bind(&DjiN3Controller::attitude_CB, this, _1));
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

    void attack(){
        dji_sdk::MFIOConfig attack_srv;
        attack_srv.request.channel = 0;
        attack_srv.request.mode = 0;
        attack_srv.request.init_on_time_us = 500;
        attack_srv.request.pwm_freq = 50;
        _attack_client.call(attack_srv);
        cout << _uavName << ": attack success!" << endl;

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


    /// ========================== 回调 ==========================
    // FLU 到 ENU 的旋转四元数订阅
    void attitude_CB(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
        quat2Tf2Rpy(msg);
    }


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
    ros::ServiceClient _attack_client;
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
