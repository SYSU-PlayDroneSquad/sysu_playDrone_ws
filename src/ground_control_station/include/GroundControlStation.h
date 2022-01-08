#ifndef SRC_GROUND_CONTROL_STATION_H
#define SRC_GROUND_CONTROL_STATION_H

// ROS include
#include <ros/ros.h>

// customer include
#include "ground_control_station/Status.h"
#include "ground_control_station/StatusArray.h"
#include "ground_control_station/Array3.h"
#include "UavStatusUpdate.h"
#include "UavVisualization.h"

using std::cout;
using std::endl;
using std::string;
using std::left;
using std::right;
using std::setw; // 对齐
using std::setprecision; // 精确
using std::vector;

class GroundControlStation{
private:
    ros::NodeHandle _nh;
    int _uavNumbers;

    UavStatusUpdate uavStatusUpdate;
    ground_control_station::Array3 _pos_arr;
    vector<UavMarker> _uav_marker_array;


    ros::Publisher _pos_pub;
    ros::Subscriber _status_sub;
    ros::Subscriber _key_sub;

    double _longitude{}, _latitude{}, _altitude{};
    double _pos_x, _pos_y, _pos_z;

    GeographicLib::LocalCartesian gps2enu;     // 调用 GeographicLib 库，转换 gps 坐标
    bool _set_local_position_ref{};

    // 高度修正值
    static vector<double> _height_corrected;


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

        //uav marker 初始化
        UavMarker marker(_nh);
        _uav_marker_array.resize(_uavNumbers, marker);

        // 设置原点
        gps2enu.Reset(latitude, longitude, altitude);


        // ros
        _pos_pub = _nh.advertise<ground_control_station::Array3>("/position_list", 10);
        _status_sub = _nh.subscribe("/UAVs/status", 10, &GroundControlStation::status_sub_cb, this);
        _key_sub = _nh.subscribe("key", 10, &GroundControlStation::key_sub_cb, this);

        // 位置
        _pos_x = _pos_y = _pos_z = 0;
        _pos_arr.x.resize(_uavNumbers); _pos_arr.y.resize(_uavNumbers); _pos_arr.z.resize(_uavNumbers);
    }

   ~GroundControlStation() = default;

    // 无人机状态回调
    void status_sub_cb(const ground_control_station::StatusArray& status){
        location_release(status); // 位置发布
        lv_gps_print(status);     // gps 姿态发布
    }

    // 键盘控制回调
    void key_sub_cb(const std_msgs::String& cmd){
        if(cmd.data == string("CorrecteHeight")){
          _height_corrected.resize(1);
          cout << "Height correcte successed!" << endl;
        }
    }

    // 位置发布
    void location_release(const ground_control_station::StatusArray &status){
        for(int i(0),j(0); i < _uavNumbers; i++){
            if(i == status.StatusArray[i].id - 1){
                _latitude = status.StatusArray[i].latitude;
                _longitude = status.StatusArray[i].longitude;
                _altitude = status.StatusArray[i].altitude;
                // 计算当前 gps 坐标对应的本地坐标
                gps2enu.Forward(_latitude, _longitude, _altitude, _pos_x, _pos_y, _pos_z);
                height_correcte(i);
                _pos_arr.x[i] = _pos_x; _pos_arr.y[i] = _pos_y; _pos_arr.z[i] = _pos_z;

                // 可视化
                //_uav_marker_array[i].handing(_pos_x, _pos_y, _pos_z, status.StatusArray[i].quaternion);
            } else {
                // cout << "uav" << status.StatusArray[j].id << "does not receive location message" << endl;
                height_correcte(i);
                _pos_arr.x[i] = 0; _pos_arr.y[i] = 0; _pos_arr.z[i] = 0;
                // 可视化
                //_uav_marker_array[i].handing(_pos_x, _pos_y, _pos_z, status.StatusArray[i].quaternion);
            }
        }
        _pos_pub.publish(_pos_arr);
    }

    // gps 状态发布
    void lv_gps_print(const ground_control_station::StatusArray &status){
        std::string data("uavx#0");
        for(int i(0); i < _uavNumbers; i++){
            if(status.StatusArray[i].id != 0){
                data[3] = status.StatusArray[i].id + '0';
                data[5] = status.StatusArray[i].lv_gps + '0';
                uavStatusUpdate.data_handing(data);
		// cout << data << endl;
            }
        }
    }

    // 高度修正
    void height_correcte(int i){
      if(_height_corrected.size() == _uavNumbers){
          _pos_z += _height_corrected[i];
      } else{
          _height_corrected.resize(i + 1);
          _height_corrected[i] = -_pos_z;
          _pos_z += _height_corrected[i];
      }
    }



};
vector<double> GroundControlStation::_height_corrected;

#endif //SRC_GROUND_CONTROL_STATION_H
