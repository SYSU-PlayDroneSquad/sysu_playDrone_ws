#ifndef SRC_GROUND_CONTROL_STATION_H
#define SRC_GROUND_CONTROL_STATION_H

// ROS include
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

// other include
#include<termios.h>
#include<sys/ioctl.h>

// customer include
#include "ground_control_station/StatusNew.h"
#include "ground_control_station/StatusArrayNew.h"
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
    int _uavNumbers; // 无人机的真实数量
    int _uavIds;     // 无人机 id 的范围

    ground_control_station::Array3 _pos_arr;
    vector<UavMarker> _uav_marker_array;

    // id 列表
    std_msgs::Int32MultiArray _id_list;
    bool _full_id_list;
    bool _get_id_list;
    double _roll, _pitch, _yaw, _yawAngle, _delta_yaw;

    ros::Publisher _pos_pub;
    ros::Publisher _id_list_pub;
    ros::Subscriber _status_sub;
    ros::Subscriber _key_sub;

    double _longitude{}, _latitude{}, _altitude{};
    double _pos_x, _pos_y, _pos_z;

    GeographicLib::LocalCartesian gps2enu;     // 调用 GeographicLib 库，转换 gps 坐标
    bool _set_local_position_ref{};

    // 高度修正值
    static vector<double> _height_corrected;

public:
    UavStatusUpdate _uavStatusUpdate;

public:
    GroundControlStation(const ros::NodeHandle& nh, const int& uavNumbers, const int& uavIds,
                         double latitude, double longitude, double altitude){
        _nh = nh;
        _uavNumbers = uavNumbers;
        _uavIds = uavIds;
        // status 初始化
        UavStatusUpdate tmp(_nh, _uavNumbers);
        UavStatusUpdate::_status.resize(_uavNumbers);
        for(int i = 0; i < _uavNumbers; i++){
            UavStatusUpdate::_status[i].resize(6);
        }
        _uavStatusUpdate = tmp;
        //uav marker 初始化
        for(int i(1); i <= _uavIds; i++){
            UavMarker marker(_nh, "uav" + std::to_string(i));
            _uav_marker_array.emplace_back(marker);
        }
        // 设置原点
        gps2enu.Reset(latitude, longitude, altitude);
        // ros
        _pos_pub = _nh.advertise<ground_control_station::Array3>("/position_list", 10);
        _status_sub = _nh.subscribe("/UAVs/status", 10, &GroundControlStation::status_sub_cb, this);
        _key_sub = _nh.subscribe("key", 10, &GroundControlStation::key_sub_cb, this);
        _id_list_pub = _nh.advertise<std_msgs::Int32MultiArray>("/id_list", 10);

        // 位置
        _pos_x = _pos_y = _pos_z = 0;
        _pos_arr.x.resize(_uavIds); _pos_arr.y.resize(_uavIds); _pos_arr.z.resize(_uavIds);
        // id列表
        _id_list.data.resize(_uavNumbers + 1); // [12(前)，1(前)，23(前)，2(后)，32(后)，0, 0, 0，0，0，2(最后一架'前'的下标)]
        _full_id_list = false;
        _get_id_list = false;
        _roll = _pitch = _yaw = _yawAngle = _delta_yaw = 0.0;
    }

    ~GroundControlStation() = default;
    GroundControlStation() = default;
    // 无人机状态回调
    void status_sub_cb(const ground_control_station::StatusArrayNew& status){
        location_release(status); // 位置发布
//        for(int i(0); i < 12 ; i++){
//          cout << "status.StatusArray[" << i << "]: " << endl;
//          cout << status.StatusArray[i] << endl;
//        }
    }

    // 键盘控制回调
    void key_sub_cb(const std_msgs::String& cmd){
        if(cmd.data == string("CorrecteHeight")){
            _height_corrected.resize(1);
            cout << "Height correcte successed!" << endl;
        } else if(cmd.data == string("GetIdList")){
            UavStatusUpdate::_output_screen = false;
            _get_id_list = !_get_id_list;
            struct winsize size{};
            ioctl(STDIN_FILENO,TIOCGWINSZ,&size);
            string tips(" Get id list");
            string line(size.ws_col, '=');
            cout << endl << line << endl << "\033[" << (size.ws_col - tips.size())/2 << "C" << tips << endl << line << endl;
        } else if(cmd.data == string("OutPutStatus")){
            UavStatusUpdate::_output_screen = !UavStatusUpdate::_output_screen;
            if(!UavStatusUpdate::_output_screen){
                struct winsize size{};
                ioctl(STDIN_FILENO,TIOCGWINSZ,&size);
                string tips(" End output status ");
                string line(size.ws_col, '=');
                cout << line << endl << "\033[" << (size.ws_col - tips.size())/2 << "C" << tips << endl << line << endl;
            }

        }
    }

    // 位置发布
    void location_release(const ground_control_station::StatusArrayNew &status){
        if(ros::isShuttingDown()) return;
        for(int i(0); i < _uavIds; i++){
            if(i == status.StatusArray[i].id - 1){
                _latitude = status.StatusArray[i].latitude;
                _longitude = status.StatusArray[i].longitude;
                _altitude = status.StatusArray[i].altitude;
                gps2enu.Forward(_latitude, _longitude, _altitude, _pos_x, _pos_y, _pos_z);     // 计算本地坐标
                height_correcte(i);                                                                     // 高度修正
                _uavStatusUpdate.data_handing(status.StatusArray[i].id, status.StatusArray[i].lv_gps,   // 状态显示
                             status.StatusArray[i].flight_status, _pos_z);
                _uav_marker_array[i].handing(_pos_x, _pos_y, _pos_z, status.StatusArray[i].quaternion); // 可视化
                _uav_marker_array[i]._motor_enable = status.StatusArray[i].flight_status;
                _pos_arr.x[i] = _pos_x; _pos_arr.y[i] = _pos_y; _pos_arr.z[i] = _pos_z;
            } else {
                if(status.StatusArray[i].id != 0){
                  ROS_ERROR("status.StatusArray[%d].id = %d", i, status.StatusArray[i].id);
                }
                height_correcte(i);
                _pos_arr.x[i] = 0; _pos_arr.y[i] = 0; _pos_arr.z[i] = 0;
            }
        }
        _pos_pub.publish(_pos_arr); // 位置发布

        if(_get_id_list){
            _id_list.data.resize(0, 0);
            _id_list.data.resize(_uavNumbers + 1, 0);
            _delta_yaw = 0.0;
            _full_id_list = false;
            // 获得flu转enu的平均偏转角
            for(int i(0); i < _uavIds; i++){
                if(i == status.StatusArray[i].id - 1){
                    geometry_msgs::QuaternionStamped quat_msg;
                    quat_msg.quaternion = status.StatusArray[i].quaternion;
                    int id = int(status.StatusArray[i].id);
                    quat2Tf2Rpy(quat_msg, id);
                } else {
                    if(status.StatusArray[i].id != 0){
                        ROS_ERROR("status.StatusArray[%d].id = %d", i, status.StatusArray[i].id);
                    }
                }
            }
            // 把 id 添加进 id_list
            int empty = 0;
            for(int i(0); i < _uavIds; i++){
                if(i == status.StatusArray[i].id - 1){
                    push_id_to_idList(int(status.StatusArray[i].id), empty);
                } else {
                    if(status.StatusArray[i].id != 0){
                        ROS_ERROR("status.StatusArray[%d].id = %d", i, status.StatusArray[i].id);
                    }
                }
            }
            // id排序
            sort(empty);

            cout << endl << "----------------------- finally sort -----------------------" << endl;
            for(int i(0); i < _uavNumbers; i++){
                cout << "uav " << _id_list.data[i] << ": = " << _pos_arr.y[_id_list.data[i] - 1] << endl;
            }
            cout << "finally_front_index = " << _id_list.data[_uavNumbers] << endl;

            _get_id_list = false;
        }
        _id_list_pub.publish(_id_list);
    }

    // 高度修正
    void height_correcte(int i){
        if(_height_corrected.size() == _uavIds){
            _pos_z += _height_corrected[i];
        } else{
          _height_corrected.resize(i + 1);
          _height_corrected[i] = -_pos_z;
          _pos_z += _height_corrected[i];
      }
    }

    // id列表
    void push_id_to_idList(int id, int &empty){
        if(!_full_id_list){
            // 检查 _id_list[] 空缺的 id 个数
            empty = 0;
            for(int i = 0; i < _uavNumbers; i++){
                if(_id_list.data[i] == 0){
                    ++empty;
                }
            }
            if(empty && !storage(id, empty)){
                _id_list.data[_uavNumbers - empty] = id;
                --empty;
                if(empty == 0){
                    _full_id_list = true;
                }
            }
        }
    }

    // 检查该 uavName 是否存储
    bool storage(const int& id, int empty) {
        for(int i = 0; i < _uavNumbers - empty; i++){
            if(_uavNumbers == empty){
                return false;
            } else if(_id_list.data[i] == id){
                return true;
            }
        }
        return false;
    }

    // id列表排序
    void sort(int empty){
        int exist = _uavNumbers - empty;
        vector<vector<double>> pos_flu;
        pos_flu.resize(exist);
        for(int i(0); i < exist; i++){
            pos_flu[i].resize(3,0);
        }

        cout << "----------------- before to_enu -----------------" << endl;
        for(int i(0); i < exist; i++){
            cout << "uav" << _id_list.data[i] << "(" << _pos_arr.x[_id_list.data[i] - 1] << ", " << _pos_arr.y[_id_list.data[i] - 1] << ")" << endl;
        }
        cout << endl;
        // enu转flu后的位置
        for(int i(0); i < exist; i++){
            pos_flu[i][0] = _id_list.data[i];
            pos_flu[i][1] = _pos_arr.x[_id_list.data[i] - 1];
            pos_flu[i][2] = _pos_arr.y[_id_list.data[i] - 1];
            pos_enu2flu(pos_flu[i][1], pos_flu[i][2]);
        }
        cout << "----------------- before sort -----------------" << endl;
        for(int i(0); i < exist; i++){
            cout << "uav" << pos_flu[i][0] << "(" << pos_flu[i][1] << ", " << pos_flu[i][2] << ")" << endl;
        }
        cout << endl;
        // 从后往前检索 id_list，按y的大小由大到小排列
        for(int n(0); n < exist; n++){
            for(int i = exist - 1; i >= 1; i--) {
                if (pos_flu[i][1] > pos_flu[i-1][1]) {
                    vector<double> tmp = {0,0,0};
                    tmp = pos_flu[i-1];
                    pos_flu[i-1] = pos_flu[i];
                    pos_flu[i] = tmp;
                }
            }
        }
        for(int i(0); i < exist; i++){
            _id_list.data[i] = int(pos_flu[i][0]);
        }

        cout << "----------------- next sort -----------------" << endl;
        for(int i(0); i < exist; i++){
            cout << "uav" << pos_flu[i][0] << "(" << pos_flu[i][1] << ", " << pos_flu[i][2] << ")" << endl;
        }

        // 记录平均机身坐标系下，在y轴上和下一架无人机间距大于 1m 的第一架无人机
        for(int i(0); i < exist - 1; i++){
            if(abs(pos_flu[i][1] - pos_flu[i+1][1]) > 3){
                _id_list.data[_uavNumbers] = i;
                cout << "The Y-axis between the Y-axis is more than 1m is uav"
                     << pos_flu[i][0] << " and uav" << pos_flu[i+1][0] << endl;
                return;
            }
        }
    }

    void quat2Tf2Rpy(const geometry_msgs::QuaternionStamped &msg, int id){
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.quaternion,quat);
        tf::Matrix3x3(quat).getRPY(_roll, _pitch, _yaw);
        cout << "uav" << id << "_angle = " << (_yaw/M_PI)*180<< "°" << endl;
        // ========================== 分割线 =============================
        if(_delta_yaw == 0.0){
            _delta_yaw = _yaw;
        }
        double angle_limit = 30;
        double angle_limit_rad = angle_limit / 180.0 * M_PI;
        double angle = abs(_delta_yaw - _yaw);

        // 死区处理
        if(_yaw < -(M_PI - angle_limit_rad) && _delta_yaw > (M_PI - angle_limit_rad)){
            angle = (M_PI - _delta_yaw) + (M_PI + _yaw);
            if(angle > angle_limit_rad){
                cout << "\033[33m[WARN]: uav" << id << " and other drones have more than " << angle_limit << "°\33[0m" << endl;
                cout << "delta_angle = " << _delta_yaw*180/M_PI << endl;
                cout << "uav" << id << "_angle = " << _yaw*180/M_PI << endl << endl;
                _yaw = _delta_yaw;
                ROS_WARN("shutdown now!");
                ros::shutdown();
                return;
            } else {
                if(_delta_yaw > abs(_yaw)){
                    _delta_yaw = (_yaw - _delta_yaw)/2;
                } else {
                    _delta_yaw = (_delta_yaw - _yaw)/2;
                }
            }
        } else if(_yaw > (M_PI - angle_limit_rad) && _delta_yaw < -(M_PI - angle_limit_rad)){
            angle = (M_PI - _yaw) + (M_PI + _delta_yaw);
            if(angle > angle_limit_rad){
                cout << "\033[33m[WARN]: uav" << id << " and other drones have more than " << angle_limit << "°\33[0m" << endl;
                cout << "delta_angle = " << _delta_yaw*180/M_PI << endl;
                cout << "uav" << id << "_angle = " << _yaw*180/M_PI << endl << endl;
                _yaw = _delta_yaw;
                ROS_WARN("shutdown now!");
                ros::shutdown();
                return;
            } else {
                if(_yaw > abs(_delta_yaw)){
                    _delta_yaw = (_delta_yaw - _yaw)/2;
                } else {
                    _delta_yaw = (_yaw - _delta_yaw)/2;
                }
            }
        } else { // 死区外
            angle = abs(_delta_yaw - _yaw);
            if(angle > angle_limit_rad){
                cout << "\033[33m[WARN]: uav" << id << " and other drones have more than " << angle_limit << "°\33[0m" << endl;
                cout << "delta_angle = " << _delta_yaw*180/M_PI << endl;
                cout << "uav" << id << "_angle = " << _yaw*180/M_PI << endl << endl;
                _yaw = _delta_yaw;
                ROS_WARN("shutdown now!");
                ros::shutdown();
            }
            _delta_yaw = (_delta_yaw + _yaw)/2;
        }

        _yawAngle = _delta_yaw * 180 / M_PI;
    }

    void pos_enu2flu(double &x, double &y){
        // 二维坐标系旋转变换
        double x_old = x;
        double y_old = y;
        cout << "angle between ENU and FLU is " << (_delta_yaw/M_PI)*180 << "°" << endl;
        x = x_old * cos(_delta_yaw) + y_old * sin(_delta_yaw);
        y = y_old * cos(_delta_yaw) - x_old * sin(_delta_yaw);

        // cout << "angle between ENU and FLU is " << (_yaw/M_PI)*180 << "°" << endl;
        // x = x_old * cos(_yaw) + y_old * sin(_yaw);
        // y = y_old * cos(_yaw) - x_old * sin(_yaw);

    }
};

#endif //SRC_GROUND_CONTROL_STATION_H
