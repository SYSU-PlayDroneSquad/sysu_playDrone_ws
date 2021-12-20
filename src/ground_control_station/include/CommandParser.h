#ifndef HECTOR_QUADROTOR_WS_UAVGROUP_H
#define HECTOR_QUADROTOR_WS_UAVGROUP_H

#include "DjiController.h"

using std::cout;
using std::endl;
using std::string;

class CommandParser{
public:
    CommandParser(ros::NodeHandle nh, string uavName){
        // 初始化无人机
        _uavName = uavName;
        DjiN3Controller uav(nh, uavName);
        _uav = uav;

    }
    ~CommandParser(){}

    void osdk_init(){
        // 获取无人机飞控型号
        _uav.get_version();
        // 获取板载控制权
        _uav.obtain_control();
        // 将本地位置的原点设置为当前 GPS 坐标
        // _uav.set_local_position();
    }

    void check_data(string data){
        string check = "empty";
        check = data.substr(0,3);
        // 判断是否为单机控制
        if(check == "uav"){
            check = data.substr(0, data.find('#'));
            cout << "uav = " << check << endl;
            // 判断控制的单机是否为本机
            if(check == _uavName){
                data = data.substr(data.find('#') + 1);
                cout << "data = " << data << endl;
                // 是本机的话直接执行指令
                baseControl(data);
            }
        } else { // 不是单机的话直接执行指令
            baseControl(data);
        }
    }
   /* 姿态信息回调
   void timerCallBack(const ros::TimerEvent &event){
        _uav.pub_status();
    }
   */

    void baseControl(const string& data) {
        if (data == "MotorUnlock") {
            _uav.motor_control();
        } else if (data == "Takeoff") {
            _uav.takeoff();
        } else if (data == "Land") {
            _uav.land();
        } else if (data == "Stop") {
            _uav.stop();
        } else if (data == "Forward") {
            _uav.forward();
        } else if (data == "Backward") {
            _uav.backward();
        } else if (data == "TurnLeft") {
            _uav.turn_left();
        } else if (data == "TurnRight") {
            _uav.turn_right();
        } else if (data == "Upward") {
            _uav.upward();
        } else if (data == "Down") {
            _uav.down();
        } else if (data == "RotateRight") {
            _uav.rotate_right();
        } else if (data == "RotateLeft") {
            _uav.rotate_left();
        } else if (data == "switchFrame") {
            _uav.switch_frame();
        } else if (data == "RoundUp") {
            _uav.round_up();
        }
        /*else if (data == "SetLocalFrame") {
            _uav.set_local_position();
        }*/
    }

private:
    string _uavName;
    DjiN3Controller _uav;

};

#endif //HECTOR_QUADROTOR_WS_UAVGROUP_H
