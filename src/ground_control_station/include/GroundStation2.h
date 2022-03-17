/*
#ifndef SRC_GROUNDSTATION2_H
#define SRC_GROUNDSTATION2_H

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QHeaderView>
#include <std_msgs/String.h>
#include "GroundControlStation.h"


namespace ground_control_station2{
    class GroundStation2: public rviz::Panel{
        // 声明Q_OBJECT宏
        Q_OBJECT
    public:
        GroundStation2(QWidget* parent = 0 );

    public Q_SLOTS:
        void updateStatus();

    protected:
        ros::NodeHandle _nh;
        QTableWidget *_uav_status_table;

        int _uavNumbers;
        GroundControlStation _groundControlStation;


    };

} // end namespace rviz_plugin_tutorials


#endif //SRC_GROUNDSTATION2_H
*/


#ifndef SRC_GROUNDSTATION2_H
#define SRC_GROUNDSTATION2_H

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QHeaderView>


namespace ground_control_station2{
    // 所有的plugin都必须是rviz::Panel的子类
    class GroundStation2: public rviz::Panel  {
        // 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
    Q_OBJECT
    public:
        // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
        GroundStation2(QWidget* parent = 0 );

        // 公共槽.
    public Q_SLOTS:
        void updateStatus(); // 更新状态
        void start();
        void center();
        void echo_keyboard_value();
        void status_output();

        // 内部变量.
    protected:
        QTableWidget *_uavStatusTable;
        QPushButton *_reset;
        QPushButton *_start;
        //创建输入框
        QLineEdit *_lineEdit;

        ros::NodeHandle _nh;


    };

} // end namespace rviz_plugin_tutorials



#endif //SRC_GROUNDSTATION2_H
