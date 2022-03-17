#ifndef HECTOR_QUADROTOR_WS_UAVGROUP_H
#define HECTOR_QUADROTOR_WS_UAVGROUP_H

#include "HectorQuadrotor.h"

using std::cout;
using std::endl;
using std::string;

template <class T>
class UavGroup{
public:
    UavGroup(ros::NodeHandle nh, int uavNumber): _uavNumbers(uavNumber) {
        // 初始化无人机组成员变量 _uav_group
        for(int i = 1; i <= _uavNumbers; i++){
            string quadrotor_name = "uav" + std::to_string(i);
            T uav(nh, quadrotor_name);
            _uav_group.emplace_back(uav);
        }
    }
    ~UavGroup(){}

    void osdk_init(const std::string dji_gazebo){
        if (dji_gazebo == "dji") {
            for(int i = 0; i < _uavNumbers; i++){
                // 获取板载控制权
                _uav_group[i].obtain_control();
                // 将本地位置的原点设置为当前 GPS 坐标
                // _uav_group[i].set_local_position();
                // 获取无人机飞控型号
                _uav_group[i].get_version();
            }
        }
    }

    void check_data(string data){
        string check = "empty";
        check = data.substr(0,3);
        // 判断是否为单机控制
        if(check == "uav"){
            check = data.substr(data.find('v') + 1, data.find('#') - data.find('v') - 1);
            // cout << "uav = " << check << endl;
            int id = std::stoi(check);
            data = data.substr(data.find('#') + 1);
            // cout << data << endl;
            singleControl(data, id - 1);
        } else if(check == "lan") {
            land_list_handling(data);
        } else if(check == "Wav"){
            cout << "uav25 start run with wave path" << endl;
            _uav_group[24].wave();
        } else { // 不是单机的话直接执行指令
            baseControl(data);
        }
    }

    void baseControl(const std::string& data){
        if (data == "MotorUnlock") {
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].motor_control();
            }
        } else if (data == "Takeoff") {
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].takeoff();
            }
        } else if (data == "Land"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].land();
            }
        } else if (data == "Stop"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].stop();
            }
        } else if (data == "Forward"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].forward();
            }
        } else if (data == "Backward"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].backward();
            }
        } else if (data == "TurnLeft"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].turn_left();
            }
        } else if (data == "TurnRight"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].turn_right();
            }
        } else if (data == "Upward"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].upward();
            }
        } else if (data == "Down"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].down();
            }
        } else if (data == "RotateRight"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].rotate_right();
            }
        } else if (data == "RotateLeft"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].rotate_left();
            }
        } else if (data == "switchFrame"){
//            for(int i = 0; i < _uavNumbers; i++){
//                _uav_group[i].switch_frame();
//            }
        } else if (data == "GoHome"){
            for(int i = 0; i < _uavNumbers; i++){
                _uav_group[i].go_home();
            }
        }
    }

    void singleControl(const std::string& data, int i){
        if (data == "MotorUnlock") {
            _uav_group[i].motor_control();
        } else if (data == "Takeoff") {
            _uav_group[i].takeoff();
        } else if (data == "Land"){
            _uav_group[i].land();
        } else if (data == "Stop"){
            _uav_group[i].stop();
        } else if (data == "Forward"){
            _uav_group[i].forward();
        } else if (data == "Backward"){
            _uav_group[i].backward();
        } else if (data == "TurnLeft"){
            _uav_group[i].turn_left();
        } else if (data == "TurnRight"){
            _uav_group[i].turn_right();
        } else if (data == "Upward"){
            _uav_group[i].upward();
        } else if (data == "Down"){
            _uav_group[i].down();
        } else if (data == "RotateRight"){
            _uav_group[i].rotate_right();
        } else if (data == "RotateLeft"){
            _uav_group[i].rotate_left();
        } else if (data == "switchFrame"){
//            for(int i = 0; i < _uavNumbers; i++){
//                _uav_group[i].switch_frame();
//            }
        } else if (data == "GoHome"){
            _uav_group[i].go_home();
        }
    }

    void land_list_handling(std::string data){
        for(int i(0); i < 4; i++){
            data = data.substr(data.find('#') + 1);
            int id = std::stoi(data.substr(0,data.find('#')));
            _uav_group[id - 1].land();
        }

    }

    void pos_globalControl(sensor_msgs::Joy pos){
        for(int i = 0; i < _uavNumbers; i++){
//            _uav_group[i].set_pos(pos);
        }
    }
    void vel_globalControl(sensor_msgs::Joy vel){
        for(int i = 0; i < _uavNumbers; i++){
//            _uav_group[i].set_vel(vel);
        }
    }
    

private:
    std::vector<T> _uav_group;
    int _uavNumbers;
};

#endif //HECTOR_QUADROTOR_WS_UAVGROUP_H
