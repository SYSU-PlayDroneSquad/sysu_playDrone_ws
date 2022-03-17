#ifndef HECTOR_QUADROTOR_WS_HECTORQUADROTOR_H
#define HECTOR_QUADROTOR_WS_HECTORQUADROTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/Joy.h>
#include <string>
//#include <Eigen/Dense>
#include <ground_control_station/EnableMotors.h>


using std::cout;
using std::endl;
using std::string;

class HectorQuadrotor {
public:
     HectorQuadrotor(ros::NodeHandle &nh, string quadrotor_name) {
         // 初始化无人机名字和无人机坐标系
         _uavName = quadrotor_name;
         _flame_world = quadrotor_name + "/world";

         // 初始化无人机位置消息
         _position_msg.header.frame_id = _flame_world;
         _home_position_msg.header.frame_id = _flame_world;
         _home_position_msg.pose.orientation.x = 0.0;
         _home_position_msg.pose.orientation.y = 0.0;
         _home_position_msg.pose.orientation.z = 0.0;
         _home_position_msg.pose.orientation.w = 1.0;
         // 初始化 home 点


         // 初始化无人机速度消息
        _twist_msg.linear.x = _twist_msg.linear.y = _twist_msg.linear.z = 0;
        _twist_msg.angular.x = _twist_msg.angular.y = _twist_msg.angular.z = 0;

        _uav_state.request.model_name = _uavName;

        _height = 4;
        if(_uavName == "uav25"){
            _height = 2;
        }
        _vel_limt = 1;
        _setp = 0.05;

        // 初始化 ros 发布器、服务器用户端
        _position_pub = nh.advertise<geometry_msgs::PoseStamped>(_uavName + "/command/pose", 1);
        _twist_pub = nh.advertise<geometry_msgs::Twist>(_uavName + "/cmd_vel", 1);
        _enable_motor_client = nh.serviceClient<ground_control_station::EnableMotors>(_uavName + "/enable_motors");
        _get_uav_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");

        /// dji接口
        _get_pos_sub = nh.subscribe(
                _uavName + "/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10, &HectorQuadrotor::get_pos_CB, this);
        _get_vel_sub = nh.subscribe<sensor_msgs::Joy>(
                _uavName + "/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10, boost::bind(&HectorQuadrotor::get_vel_CB,this, _1, _twist_pub));

    }
    ~HectorQuadrotor() {}

    /// 键盘控制
    void motor_control() {
        ground_control_station::EnableMotors srv;
        srv.request.enable = true;
        _enable_motor_client.call(srv);
        cout << _uavName << " enable success!" << endl;
    }

    void takeoff() {
        // 获取无人机当前位置和速度信息
        _get_uav_state_client.call(_uav_state);
        // 当前位置赋值
        _position_msg.pose.position.x = _uav_state.response.pose.position.x;
        _position_msg.pose.position.y = _uav_state.response.pose.position.y;
        // home 点赋值
        _home_position_msg.pose.position.x = _uav_state.response.pose.position.x;
        _home_position_msg.pose.position.y = _uav_state.response.pose.position.y;
        _home_position_msg.pose.position.z = _height;
        if(_uavName == "uav25"){
            _home_position_msg.pose.position.z = 2;
        }

        // 起飞
        if (_uav_state.response.pose.position.z < 0.3) {
            _position_msg.pose.position.z = _height;
            _position_pub.publish(_position_msg);
        }
        cout << _uavName << " send takeoff success!" << endl;
    }

    void land() {
        // 获取无人机当前位置和速度信息
        _get_uav_state_client.call(_uav_state);
        // 当前位置赋值
        _position_msg.pose.position.x = _uav_state.response.pose.position.x;
        _position_msg.pose.position.y = _uav_state.response.pose.position.y;
        // 降落
        _position_msg.pose.position.z = 0;
        _position_msg.header.frame_id = _flame_world;
        _position_pub.publish(_position_msg);
        cout << _uavName << " send land success!" << endl;
    }

    void stop() { // 悬停
        _twist_msg.linear.x = _twist_msg.linear.y = _twist_msg.linear.z = 0;
        _twist_msg.angular.x = _twist_msg.angular.y = _twist_msg.angular.z = 0;
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send stop success!" << endl;
    }

    void forward() {
        if (_twist_msg.linear.x >= 0 && _twist_msg.linear.x < _vel_limt) {
            _twist_msg.linear.x += _setp;
        } else if (_twist_msg.linear.x < 0) {
            _twist_msg.linear.x = 0;
        } else {
            _twist_msg.linear.x = _vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send forward success!" << endl;
    }

    void backward() {
        if (_twist_msg.linear.x <= 0 && _twist_msg.linear.x > -_vel_limt) {
            _twist_msg.linear.x -= _setp;
        } else if (_twist_msg.linear.x > 0) {
            _twist_msg.linear.x = 0;
        } else {
            _twist_msg.linear.x = -_vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send backward success!" << endl;
    }

    void turn_right() {
        if (_twist_msg.linear.y <= 0 && _twist_msg.linear.y > -_vel_limt) {
            _twist_msg.linear.y -= _setp;
        } else if (_twist_msg.linear.y > 0) {
            _twist_msg.linear.y = 0;
        } else {
            _twist_msg.linear.y = -_vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send turn_left success!" << endl;
    }

    void turn_left() {
        if (_twist_msg.linear.y >= 0 && _twist_msg.linear.y < _vel_limt) {
            _twist_msg.linear.y += _setp;
        } else if (_twist_msg.linear.y < 0) {
            _twist_msg.linear.y = 0;
        } else {
            _twist_msg.linear.y = _vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send turn_right success!" << endl;
    }

    void upward() {
        if (_twist_msg.linear.z >= 0 && _twist_msg.linear.z < _vel_limt) {
            _twist_msg.linear.z += _setp;
        } else if (_twist_msg.linear.z < 0) {
            _twist_msg.linear.z = 0;
        } else {
            _twist_msg.linear.z = _vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send upward success!" << endl;
    }

    void down() {
        if (_twist_msg.linear.z <= 0 && _twist_msg.linear.z > -_vel_limt) {
            _twist_msg.linear.z -= _setp;
        } else if (_twist_msg.linear.z > 0) {
            _twist_msg.linear.z = 0;
        } else {
            _twist_msg.linear.z = -_vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send down success!" << endl;
    }

    void rotate_left() {
        if (_twist_msg.angular.z >= 0 && _twist_msg.linear.z < _vel_limt) {
            _twist_msg.angular.z += _setp;
        } else if (_twist_msg.angular.z < 0) {
            _twist_msg.angular.z = 0;
        } else {
            _twist_msg.angular.z = _vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send rotate_right success!" << endl;
    }

    void rotate_right() {
        if (_twist_msg.angular.z <= 0 && _twist_msg.angular.z > -_vel_limt) {
            _twist_msg.angular.z -= _setp;
        } else if (_twist_msg.angular.z > 0) {
            _twist_msg.angular.z = 0;
        } else {
            _twist_msg.angular.z = -_vel_limt;
        }
        _twist_pub.publish(_twist_msg);
        cout << _uavName << " send rotate_right success!" << endl;
    }

    void wave(){
        ros::Time begin = ros::Time::now();
        ros::Rate r(10);
        double v_x = 1;
        double v_y = 1;
        while(ros::ok()){
            ros::Time now = ros::Time::now();
            ros::Duration delta_t = now - begin;
            double t = delta_t.toSec();
            _twist_msg.linear.x = v_x * cos(t * M_PI/6);
            _twist_msg.linear.y = abs(v_y * sin(t * M_PI/6));
            _twist_pub.publish(_twist_msg);
            if(t > 18){
                _twist_msg.linear.x = 0;
                _twist_msg.linear.y = 0;
                 _twist_pub.publish(_twist_msg);
                return;
            }
            r.sleep();
        }
        cout << "uav start run with wave path" << endl;


     }

    void set_pos(sensor_msgs::Joy pos){
        _position_msg.pose.position.x = pos.axes[0];
        _position_msg.pose.position.y = pos.axes[1];
        _position_msg.pose.position.z = pos.axes[2];
        _position_pub.publish(_position_msg);
     }

    void set_vel(sensor_msgs::Joy vel){
         _twist_msg.linear.x = vel.axes[0];
         _twist_msg.linear.y = vel.axes[1];
         _twist_msg.linear.z = vel.axes[2];
         _twist_msg.angular.z = vel.axes[3];
         _position_pub.publish(_twist_msg);
     }

     void go_home(){
        _position_pub.publish(_home_position_msg);
         cout << _uavName << " send go_home success!" << endl;
     }

    /// 辅助函数
    void get_rate(){}

    /// 位置速度
    void get_pos_CB(const sensor_msgs::Joy &msg){

     }

    void get_vel_CB(const sensor_msgs::Joy::ConstPtr &msg, ros::Publisher &vel_pub){
        geometry_msgs::Twist vel;

        vel.linear.x = msg->axes[0];
        vel.linear.y = msg->axes[1];
        vel.linear.z = msg->axes[2];
        vel.angular.z = msg->axes[3];
        cout << "vel:\n" << vel << endl;
        vel_pub.publish(vel);
    }

    // 机身(FLU)&大地(ENU)坐标系切换
    void switch_frame(){
    }


private:
    string _uavName;
    string _flame_world;
    int _height;
    int _vel_limt;
    double _setp;


    /// gazebo 仿真
    //  位置发布
    ros::Publisher _position_pub;
    geometry_msgs::PoseStamped _position_msg;
    geometry_msgs::PoseStamped _home_position_msg;
    //  速度发布
    ros::Publisher _twist_pub;
    geometry_msgs::Twist _twist_msg;
    //  获取 gazebo 模型的位置、速度等状态信息
    ros::ServiceClient _get_uav_state_client;
    gazebo_msgs::GetModelState _uav_state;
    //  使能电机
    ros::ServiceClient _enable_motor_client;

    /// dji接口
    ros::Subscriber _get_vel_sub;
    ros::Subscriber _get_pos_sub;

    static double _xCmd, _yCmd, _zCmd, _yawCmd;

};

double HectorQuadrotor::_xCmd = 0.0;
double HectorQuadrotor::_yCmd = 0.0;
double HectorQuadrotor::_zCmd = 0.0;
double HectorQuadrotor::_yawCmd = 0.0;



#endif //HECTOR_QUADROTOR_WS_HECTORQUADROTOR_H
