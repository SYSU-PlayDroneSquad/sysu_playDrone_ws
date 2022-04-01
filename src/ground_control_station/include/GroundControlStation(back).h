#ifndef SRC_GROUND_CONTROL_STATION_H
#define SRC_GROUND_CONTROL_STATION_H

// ROS include
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

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
            _get_id_list = !_get_id_list;
            cout << "get_id_list = " << _get_id_list << endl;
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
            _id_list.data.resize(_uavNumbers, 0);
            _delta_yaw = 0.0;
            _full_id_list = false;
            cout << "=============================== start ===============================" << endl;
            cout << "init: " << endl << _id_list << endl;
            // 得出flu转enu的平均偏转角
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
            cout << "detal_yaw = " << _delta_yaw << endl;
            cout << "yawangle = " << _yawAngle << endl;

            /*cout << endl << "==================== pos_arr =================== "<< endl;
            for(int i(0); i < _uavIds; i++){
                cout << "uav " << i + 1 << ": y = " << _pos_arr.y[i] << endl;
            }
            cout << endl << "========================= id_list ========================" << endl;
            for(int i(0),j(1); i < _uavNumbers; i++){
                _pos_arr.y[i] = j;
                j++;
                cout << "uav " << _id_list.data[i] << ": = " << _pos_arr.y[_id_list.data[i] - 1] << endl;
            }*/
//            cout << endl << "----------------------- current -----------------------" << endl;

            // 把 id 添加进 id_list
            int empty = 0;
            for(int i(0); i < _uavIds; i++){
                if(i == status.StatusArray[i].id - 1){
                    push_id_to_idList(status.StatusArray[i].id, empty);
                } else {
                    if(status.StatusArray[i].id != 0){
                        ROS_ERROR("status.StatusArray[%d].id = %d", i, status.StatusArray[i].id);
                    }
                }
            }

            sort(empty);

            cout << endl << "----------------------- finally sort -----------------------" << endl;
            for(int i(0); i < _uavNumbers + 1; i++){
                cout << "uav " << _id_list.data[i] << ": = " << _pos_arr.y[_id_list.data[i] - 1] << endl;
            }

            cout << "========================== end ==========================" << endl;
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
                //sort(empty);
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
        // 假定位置
        cout << endl << "----------------------- before sort -----------------------" << endl;
        cout << "_delta_yaw = " << _delta_yaw << endl;
        cout << "yaw angle = " << _yawAngle << endl;
        _id_list.data[0] = 3; _id_list.data[1] = 5; _id_list.data[2] = 2; _id_list.data[3] = 8;
        _pos_arr.x[2] = -1.06066; _pos_arr.y[2] = -1.06066;
        _pos_arr.x[4] = 1.06066; _pos_arr.y[4] = 1.06066;
        _pos_arr.x[1] = -3.18198; _pos_arr.y[1] = 1.06066;
        _pos_arr.x[7] = -1.06066; _pos_arr.y[7] = 3.18198;

        // enu转flu后的位置
        for(int i(0); i < exist; i++){
            pos_flu[i][0] = _id_list.data[i];
            pos_flu[i][1] = _pos_arr.x[_id_list.data[i] - 1];
            pos_flu[i][2] = _pos_arr.y[_id_list.data[i] - 1];
            pos_enu2flu(pos_flu[i][1], pos_flu[i][2]);
            cout << "uav" << _id_list.data[i] << "(" << _pos_arr.x[_id_list.data[i] - 1] << "," << _pos_arr.y[_id_list.data[i] - 1] << ")"<< endl;
            cout << "uav" << _id_list.data[i] << "(" << pos_flu[i][1] << "," << pos_flu[i][2] << ")"<< endl << endl;
        }
        // 从后往前检索 id_list，按y的大小由大到小排列
        for(int n(0); n < exist; n++){
            for(int i = exist - 1; i >= 1; i--) {
                if (pos_flu[i][2] > pos_flu[i-1][2]) {
                    vector<double> tmp = {0,0,0};
                    tmp = pos_flu[i-1];
                    pos_flu[i-1] = pos_flu[i];
                    pos_flu[i] = tmp;
                }
            }
            cout << endl << "----------------------- next sort -----------------------" << endl;
            for(int i(0); i < exist; i++){
                cout << "uav " << pos_flu[i][0] << ": = " << pos_flu[i][2] << endl;
            }
        }
        for(int i(0); i < exist; i++){
            _id_list.data[i] = int(pos_flu[i][0]);
        }
        // 记录平均机身坐标系下，在y轴上和下一架无人机间距大于 1m 的第一架无人机
        for(int i(0); i < exist - 1; i++){
            if(abs(pos_flu[i][2] - pos_flu[i+1][2]) > 1){
                _id_list.data[_uavNumbers] = int(pos_flu[i][0]);
                cout << "The Y-axis between the Y-axis is more than 1m is uav"
                     << pos_flu[i][0] << " and uav" << pos_flu[i+1][0] << endl;
                return;
            }
        }
    }

    // 赋值速度消息


    void quat2Tf2Rpy(const geometry_msgs::QuaternionStamped &msg, int id){
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.quaternion,quat);
        tf::Matrix3x3(quat).getRPY(_roll, _pitch, _yaw);

        if(_delta_yaw == 0.0){
            _delta_yaw = _yaw;
        }
        int angle = 30;
        double delta_angle = angle / 180.0 * M_PI;

        if(abs(_delta_yaw - _yaw) > delta_angle){
            cout << "\033[33m[WARN]: uav" << id << " and other drones have more than " << angle << "°\33[0m" << endl;
            cout << "delta_angle = " << _delta_yaw*180/M_PI << endl;
            cout << "uav" << id << "_angle = " << _yaw*180/M_PI << endl << endl;
            _yaw = _delta_yaw;
            ROS_WARN("shutdown now!");
            ros::shutdown();
        }
        _delta_yaw = (_delta_yaw + _yaw)/2;
        _yawAngle = _delta_yaw * 180 / M_PI;
    }

    void pos_enu2flu(double &x, double &y){//速度控制转换
        // 二维坐标系旋转变换
        double x_old = x;
        double y_old = y;
        x = x_old * cos(-_delta_yaw) + y_old * sin(-_delta_yaw);
        y = y_old * cos(-_delta_yaw) - x_old * sin(-_delta_yaw);

    }
};

#endif //SRC_GROUND_CONTROL_STATION_H
