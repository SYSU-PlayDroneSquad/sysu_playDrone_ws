#ifndef SRC_UAVVISUALIZATION_H
#define SRC_UAVVISUALIZATION_H

// ros include
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

// other include
#include <GeographicLib/LocalCartesian.hpp>
#include<iomanip>
#include <utility>
// customer include

using std::cout;
using std::endl;
using std::string;
using std::left;
using std::right;
using std::setw; // 对齐
using std::setprecision; // 精确

class UavMarker{
private:
    string _uavName;
    double _prop_radius;                 // 螺旋桨半径
    double _arm_length;                  // 无人机臂长
    string _shape_frame;                 // 机架类型
    static double _setp_angle;           // 螺旋桨步进角度
    double _roll, _pitch, _yaw;          // 机体坐标系到 ENU 坐标系的姿态角变换
    visualization_msgs::MarkerArray  _uav_marker_msg; // 无人机显示消息
    static nav_msgs::Path _path_msg;            // 运动路径

    // ros通信
    ros::NodeHandle _nh;
    ros::Publisher _marker_pub;
    ros::Publisher _path_pub;
    ros::Timer _motor_setup = _nh.createTimer(ros::Duration(0.05), &UavMarker::rps_CB, this);

public:
    UavMarker(ros::NodeHandle &nh, string uavName){
//        _nh = nh;
        _uavName = std::move(uavName);
        _prop_radius = 0.065;
        _arm_length = 0.3;
        _shape_frame = "x"; // 十形、X形
        _roll = _pitch = _yaw = 0;

        _path_msg.header.frame_id = "map";

        _marker_pub  = nh.advertise<visualization_msgs::MarkerArray>(_uavName + "/marker", 10);
        _path_pub = nh.advertise<nav_msgs::Path>(_uavName + "/path", 10);
        init_uav_markers();
    }
    ~UavMarker() = default;
    static bool _motor_enable;

    // 初始化无人机 marker
    void init_uav_markers(){
        _uav_marker_msg.markers.resize(4);
        // 定义螺旋桨
        visualization_msgs::Marker propeller;
        propeller.header.frame_id = "map";
        propeller.type = visualization_msgs::Marker::CYLINDER;
        propeller.action = visualization_msgs::Marker::MODIFY;
        propeller.scale.x = 2 * _prop_radius;
        propeller.scale.y = 0.2 * _prop_radius;
        propeller.scale.z = 0.1 * _prop_radius;
        propeller.color.a = 1;
        propeller.color.r = 0;
        propeller.color.g = 0;
        propeller.color.b = 0;
        for (int i=0; i<4; ++i) {
            _uav_marker_msg.markers[i] = propeller;
            _uav_marker_msg.markers[i].id = i;
            switch (i) {
                case 0:  _uav_marker_msg.markers[0].color.r = 0.784; break;
                case 1:  _uav_marker_msg.markers[1].color.g = 0.784; break;
                case 2:  _uav_marker_msg.markers[2].color.b = 0.784; break;
                default: break;
            }
        }
        // 定义机体
        visualization_msgs::Marker drone_body = propeller;
        drone_body.type = visualization_msgs::Marker::SPHERE;
        drone_body.scale.x =  _arm_length;
        drone_body.scale.y = 0.2 *  _arm_length;
        drone_body.scale.z = 0.1 *  _arm_length;
        _uav_marker_msg.markers.push_back(drone_body);
        drone_body.scale.y =  _arm_length;
        drone_body.scale.x = 0.2 *  _arm_length;
        drone_body.color.r =  drone_body.color.g =  drone_body.color.b =1;
        _uav_marker_msg.markers.push_back(drone_body);
    }

    void handing(double pos_x, double pos_y, double pos_z, geometry_msgs::Quaternion quaternion){
        get_uav_markers(pos_x, pos_y, pos_z, quaternion);
        if(_uavName == "uav1"){
          get_path(pos_x, pos_y, pos_z);
        }

    }
    // 飞行路径
    void get_path(double pos_x, double pos_y, double pos_z){
        geometry_msgs::PoseStamped this_pose;
        this_pose.header.stamp = ros::Time::now();
        this_pose.header.frame_id = "map";
        this_pose.pose.position.x = pos_x;
        this_pose.pose.position.y = pos_y;
        this_pose.pose.position.z = pos_z;
        this_pose.pose.orientation.x = 0.0;
        this_pose.pose.orientation.y = 0.0;
        this_pose.pose.orientation.z = 0.0;
        this_pose.pose.orientation.w = 0.0;
        _path_msg.poses.emplace_back(this_pose);
        _path_pub.publish(_path_msg);
    }

    // 获取无人机的位置消息
    void get_uav_markers(double pos_x, double pos_y, double pos_z, geometry_msgs::Quaternion quaternion){
        // 机体的位置姿态
        geometry_msgs::Pose drone_pose;
        drone_pose.position.x = pos_x;
        drone_pose.position.y = pos_y;
        drone_pose.position.z = pos_z;
        drone_pose.orientation.x = quaternion.x;
        drone_pose.orientation.y = quaternion.y;
        drone_pose.orientation.z = quaternion.z;
        drone_pose.orientation.w = quaternion.w;

        tf::Quaternion quat;
        // cout << drone_pose.orientation << endl;
        tf::quaternionMsgToTF(drone_pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(_roll, _pitch, _yaw); // 四元数转 rpy

        if(_shape_frame == "x"){
            // Tb = Ta * Tab
            // Ta:
            tf::Matrix3x3 drone_p;
            drone_p.setRotation(quat); // 四元数转旋转矩阵

            // Tab:
            geometry_msgs::Quaternion tmp_tf_quat;
            tmp_tf_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI/4); // rpy 转四元数
            tf::quaternionMsgToTF(tmp_tf_quat,quat); // 四元数转tf四元数
            tf::Matrix3x3 t_f;
            t_f.setRotation(quat); // 四元数转旋转矩阵

            // Tb:
            tf::Matrix3x3 result = drone_p * t_f;
            result.getRPY(_roll, _pitch, _yaw); // 旋转矩阵转rpy
        }
        //cout << "yawrate = " << _yaw * 180/M_PI << endl;

        drone_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(_roll, _pitch, _yaw);// rpy 转四元数


        for(int i=4; i < 6; ++i) {
            _uav_marker_msg.markers[i].pose = drone_pose;
            _uav_marker_msg.markers[i].id = i;
        }
        // ================================= 螺旋桨的位置姿态 =================================
        // 螺旋桨的位置姿态
        geometry_msgs::Pose propeller_pose;

        // 螺旋桨姿态
        propeller_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(_roll, _pitch, _setp_angle);// rpy 转四元数

        // 螺旋桨位置
        get_propeller_pos(drone_pose, propeller_pose);

        _marker_pub.publish(_uav_marker_msg);
    }

    // 获取螺旋桨的位置
    void get_propeller_pos(geometry_msgs::Pose drone_pose, geometry_msgs::Pose propeller_pose){
        propeller_pose.position.z = drone_pose.position.z + 0.025;
        for(int i = 0; i < 4; i++){
            switch (i) {
                case 0:
                    propeller_pose.position.x = drone_pose.position.x + _arm_length/2 * cos(-_yaw);
                    propeller_pose.position.y = drone_pose.position.y - _arm_length/2 * sin(-_yaw);
                    break;
                case 1:
                    propeller_pose.position.x = drone_pose.position.x + _arm_length/2 * sin(-_yaw);
                    propeller_pose.position.y = drone_pose.position.y + _arm_length/2 * cos(-_yaw);
                    break;
                case 2:
                    propeller_pose.position.x = drone_pose.position.x - _arm_length/2 * cos(-_yaw);
                    propeller_pose.position.y = drone_pose.position.y + _arm_length/2 * sin(-_yaw);
                    break;
                case 3:
                    propeller_pose.position.x = drone_pose.position.x - _arm_length/2 * sin(-_yaw);
                    propeller_pose.position.y = drone_pose.position.y - _arm_length/2 * cos(-_yaw);
                    break;
                default: break;

            }
            _uav_marker_msg.markers[i].pose = propeller_pose;
        }
    }

    // 螺旋桨步进角
    void rps_CB(const ros::TimerEvent &event){
        if(_motor_enable){
            _setp_angle += M_PI/10; // 2π = 360°
        }
    }
};


#endif //SRC_UAVVISUALIZATION_H
