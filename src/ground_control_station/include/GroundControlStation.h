#ifndef SRC_GROUND_CONTROL_STATION_H
#define SRC_GROUND_CONTROL_STATION_H

// ROS include
#include <ros/ros.h>

// customer include
#include "ground_control_station/Status.h"
#include "ground_control_station/StatusArray.h"
#include "ground_control_station/Array3.h"
#include "UavStatusUpdate.h"

using std::cout;
using std::endl;
using std::string;
using std::left;
using std::right;
using std::setw; // 对齐
using std::setprecision; // 精确

class GroundControlStation{
private:
    ros::NodeHandle _nh;
    int _uavNumbers;

    UavStatusUpdate uavStatusUpdate;

    ros::Publisher _pos_pub;
    ros::Subscriber _status_sub;

    double _longitude{}, _latitude{}, _altitude{};
    double _pos_x, _pos_y, _pos_z;
    ground_control_station::Array3 _pos_arr;
    GeographicLib::LocalCartesian gps2enu;     // 调用 GeographicLib 库，转换 gps 坐标
    bool _set_local_position_ref{};


public:
    GroundControlStation(const ros::NodeHandle& nh, const int& uavNumbers,
                         double latitude, double longitude, double altitude){
        _nh = nh;
        _uavNumbers = uavNumbers;
        // status 初始化
        UavStatusUpdate tmp(_nh, _uavNumbers);
        UavStatusUpdate::_status.resize(_uavNumbers);
        for(int i = 0; i < _uavNumbers; i++){
            UavStatusUpdate::_status[i].resize(3);
        }
        uavStatusUpdate = tmp;

        // 设置原点
        gps2enu.Reset(latitude, longitude, altitude);
        // ros
        _pos_pub = _nh.advertise<ground_control_station::Array3>("/position_list", 10);
        _status_sub = _nh.subscribe("/UAVs/status", 1, &GroundControlStation::status_sub_cb, this);

        _pos_x = _pos_y = _pos_z = 0;
        _pos_arr.x.resize(_uavNumbers); _pos_arr.y.resize(_uavNumbers); _pos_arr.z.resize(_uavNumbers);
    }

   ~GroundControlStation() = default;

    // 无人机状态回调
    void status_sub_cb(const ground_control_station::StatusArray& status){
        location_release(status); // 位置发布
        lv_gps_print(status);
    }

    // 位置发布
    void location_release(const ground_control_station::StatusArray &status){
        for(int i(0); i < _uavNumbers; i++){
            _latitude = status.StatusArray[i].latitude;
            _longitude = status.StatusArray[i].longitude;
            _altitude = status.StatusArray[i].altitude;
            // 计算当前 gps 坐标对应的本地坐标
            gps2enu.Forward(_latitude, _longitude, _altitude, _pos_x, _pos_y, _pos_z);
            _pos_arr.x[i] = _pos_x; _pos_arr.y[i] = _pos_y; _pos_arr.z[i] = _pos_z;
        }
        _pos_pub.publish(_pos_arr);
    }

    // gps 状态发布
    void lv_gps_print(const ground_control_station::StatusArray &status){
        std::string data("uavx#0");
        for(int i(0); i < _uavNumbers; i++){
            data[3] = status.StatusArray[i].id + '0';
            data[5] = status.StatusArray[i].lv_gps + '0';
            uavStatusUpdate.data_handing(data);
        }

    }

};


#endif //SRC_GROUND_CONTROL_STATION_H
