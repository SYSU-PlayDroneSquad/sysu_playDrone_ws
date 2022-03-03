#ifndef SRC_UAVSTATUSUPDATE_H
#define SRC_UAVSTATUSUPDATE_H

// ROS include
#include <ros/ros.h>
#include <std_msgs/String.h>

// c++ include
#include <map>
#include <sstream>

using namespace std;

class UavStatusUpdate{
public:
    UavStatusUpdate() = default;
    UavStatusUpdate(const ros::NodeHandle& nh, int uavNumbers) {
        // ros句柄
        _nh = nh;
        _uavNumbers = uavNumbers;
        _full = false;
        string space = " "; space.resize(8, ' ');
        _lv_5 = "\33[31m▮\33[35m▮\33[33m▮\33[32m▮▮\33[0m" + space;
        _lv_4 = "\33[31m▮\33[35m▮\33[33m▮\33[32m▮\33[0m " + space;
        _lv_3 = "\33[31m▮\33[35m▮\33[33m▮\33[0m  " + space;
        _lv_2 = "\33[31m▮\33[35m▮\33[0m   " + space;
        _lv_1 = "\33[31m▮\33[0m    " + space;
        _lv_0 = "\33[31m✖\33[0m    " + space;

    }

    ~UavStatusUpdate()= default;

    void data_handing(unsigned int id, unsigned int lv_gps, unsigned int flight_status, double &height){
        push_uavName(id);
        push_item(id, lv_gps, height, flight_status);
        // output();
    }

    // 把 uavName push 进 _status
    void push_uavName(unsigned int id){
        if(!_full){
            // 检查 _status[] 空缺的uavName个数
            int empty = 0;
            for(int i = 0; i < _uavNumbers; i++){
                if(_status[i][0].empty()){
                    ++empty;
                }
            }
            string uavName = "uav " + std::to_string(id);
            if(empty && !storage(uavName, empty)){
                _status[_uavNumbers - empty][0] = uavName;
                --empty;
                // cout << "empty2 = " << empty << endl;
                if(empty == 0){
                    _full = true;
                }
                sort(empty);
            }
        }
    }


    // 检查该 uavName 是否存储
    bool storage(const string& uavName, int empty) const{
        for(int i = 0; i < _uavNumbers - empty; i++){
            if(_uavNumbers == empty){
                return false;
            } else if(_status[i][0] == uavName){
                return true;
            }
        }
        return false;
    }

    // 对 _status 里的 uavName 升序排序
    void sort(int empty) const{
        string tmp;
        int exist = _uavNumbers - empty;
        for(int i = exist - 1; i >= 1; i--){
            if(_status[i][0] < _status[i - 1][0]){
                tmp = _status[i][0];
                _status[i][0] = _status[i - 1][0];
                _status[i - 1][0] = tmp;

                tmp = _status[i][1];
                _status[i][1] = _status[i - 1][1];
                _status[i - 1][1] = tmp;

                tmp = _status[i][2];
                _status[i][2] = _status[i - 1][2];
                _status[i - 1][2] = tmp;
            }
        }
    }


    // 把无人机的状态条目 push 到 _status
    void push_item(unsigned int &id, unsigned int &lv_gps, double &height, unsigned int flight_status) const{
        string uavName = "uav " + std::to_string(id);
        string icon_gps;
        if(height < 0.1) height = 0.00;

        switch (lv_gps) {
            case 5: icon_gps = _lv_5; break;
            case 4: icon_gps = _lv_4; break;
            case 3: icon_gps = _lv_3; break;
            case 2: icon_gps = _lv_2; break;
            case 1: icon_gps = _lv_1; break;
            case 0: icon_gps = _lv_0; break;
            default: icon_gps = "???";break;
        }
        for(int i = 0; i < _uavNumbers; i++){
            if(uavName == _status[i][0]){
                _status[i][2] = icon_gps;
                _status[i][3] = precision(height);
                _status[i][4] = motor_status(flight_status);
                int count = atoi(_status[i][1].c_str());
                count++;
                _status[i][1] = std::to_string(count);
            }
        }
    }

    // 数据精度
    string precision(const double &value) const {
        std::stringstream ss;
        ss << setprecision(3) << value; // 保留 3 位有效数字
        string result;
        return ss.str();
    }

    // 电机状态
    string motor_status(unsigned int &flight_status) const {
        string motor_status;
        switch (flight_status){
            case 0: motor_status = "\33[31moff\33[0m"; break;
            case 1: motor_status = "\33[32mon\33[0m"; break;
            case 2: motor_status = "\33[32minflight\33[0m"; break;
            default: motor_status = " "; break;
        }
        return motor_status;
    }


    // 输出到屏幕
    void output() const{
        system("clear");
        string header =  "编号        GPS         高度(m)      电机    ";
        string line   =  "——————————————————————————————————————————————";
        string flash  =  "\33[37m☁\33[0m";

        switch (_flash++) {
            case 0: cout << header << endl << line << endl; break;
            case 1: cout << header << flash << endl << line << endl; _flash = 0; break;
        }

        int empty = 0;
        for(int i = 0; i < _uavNumbers; i++){
            if(!_status[i][0].empty()){
                 cout << left << setw(12) << _status[i][0] << _status[i][2] << setw(12) << _status[i][3] << _status[i][4] << endl;
                // cout << _status[i][3] << endl;
            } else {
                empty++;
            }
        }
        if(empty){
            printf("\n\33[33m%d\33[0m uavs are not connected\n", empty);
        }
    }

    // 检查链接通断
    void check_status_CB(const ros::TimerEvent &event) const{
        for(int i = 0; i < _uavNumbers; i++){
            if(!_status[i][0].empty()){
                if(_status[i][1] < "1"){
                    _status[i][2] = "\33[31m        Link  down!       \33[0m";
                    _status[i][3] = " ";
                    _status[i][4] = " ";
                }
                _status[i][1] = "0";
                output();
            }
        }
    }

public:
    static vector<vector<string>> _status;

private:
    ros::NodeHandle _nh;
    int _uavNumbers{};
    bool _full{};
    static int _flash;
    string _lv_0; string _lv_1; string _lv_2;
    string _lv_3; string _lv_4; string _lv_5;
    ros::Timer _check_status =
            _nh.createTimer(ros::Duration(0.1), &UavStatusUpdate::check_status_CB, this);
};

vector<vector<string>> UavStatusUpdate::_status(1);
int UavStatusUpdate::_flash = 0;

#endif //SRC_UAVSTATUSUPDATE_H
