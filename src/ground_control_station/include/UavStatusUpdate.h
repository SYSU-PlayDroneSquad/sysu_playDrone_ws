#ifndef SRC_UAVSTATUSUPDATE_H
#define SRC_UAVSTATUSUPDATE_H

// ROS include
#include <ros/ros.h>
#include <std_msgs/String.h>

// c++ include
#include <map>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::left;
using std::right;
using std::setw;

class UavStatusUpdate{
public:
    UavStatusUpdate(const ros::NodeHandle& nh, int uavNum) {
        // ros句柄
        _nh = nh;
        _uavNum = uavNum;
        _full = false;
    }

    ~UavStatusUpdate()= default;

    void data_handing(const string& data){
        push_uavName(data);
        push_item(data);
        output();
    }

    // 把 uavName push 进 _status
    void push_uavName(string data){
        if(!_full){
            // 检查 _status[] 空缺的uavName个数
            int empty = 0;
            for(int i = 0; i < _uavNum; i++){
                if(_status[i][0].empty()){
                    ++empty;
                }
            }
            string uavName = data.substr(0, data.find('v') + 1) + " "
                    + data.substr(data.find('v') + 1, data.find('#') - data.find('v') - 1);
            if(empty && !storage(uavName, empty)){
                _status[_uavNum - empty][0] = uavName;
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
        for(int i = 0; i < _uavNum - empty; i++){
            if(_uavNum == empty){
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
        int exist = _uavNum - empty;
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
    void push_item(string data) const{
        string uavName = data.substr(0, data.find('v') + 1) + " "
                         + data.substr(data.find('v') + 1, data.find('#') - data.find('v') - 1);
        int lv_gps = data[data.find('#') + 1] - '0';

        string icon_gps;
        switch (lv_gps) {
            case 5: icon_gps = "\33[31m▮\33[35m▮\33[33m▮\33[32m▮▮\33[0m"; break;
            case 4: icon_gps = "\33[31m▮\33[35m▮\33[33m▮\33[32m▮\33[0m";  break;
            case 3: icon_gps = "\33[31m▮\33[35m▮\33[33m▮\33[0m";    break;
            case 2: icon_gps = "\33[31m▮\33[35m▮\33[0m";     break;
            case 1: icon_gps = "\33[31m▮\33[0m";       break;
            case 0: icon_gps = "\33[31m✖\33[0m"; break;
            default: icon_gps = "???";    break;
        }
        for(int i = 0; i < _uavNum; i++){
            if(uavName == _status[i][0]){
                _status[i][2] = icon_gps;
                int count = atoi(_status[i][1].c_str());
                count++;
                _status[i][1] = std::to_string(count);
            }
        }
    }

    // 输出到屏幕
    void output() const{
        system("clear");
        string header =  "编号          GPS           位置           ";
        string line   =  "————————————————————————————————————————————";
        string flash  =  "\33[37m☁\33[0m";

        switch (_flash++) {
            case 0: cout << header << endl << line << endl; break;
            case 1: cout << header << flash << endl << line << endl; _flash = 0; break;
        }

        int empty = 0;
        for(int i = 0; i < _uavNum; i++){
            if(!_status[i][0].empty()){
                cout << left << setw(13) << _status[i][0] << _status[i][2] << endl;
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
        for(int i = 0; i < _uavNum; i++){
            if(!_status[i][0].empty()){
                if(_status[i][1] < "3"){
                    _status[i][2] = "\33[31mLink down!\33[0m";
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
    int _uavNum;
    bool _full;
    static int _flash;
};

vector<vector<string>> UavStatusUpdate::_status(1);
int UavStatusUpdate::_flash = 0;

#endif //SRC_UAVSTATUSUPDATE_H
