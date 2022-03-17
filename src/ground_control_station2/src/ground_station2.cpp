#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QtWidgets/QTableWidget>
#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "../include/GroundStation2.h"

namespace ground_control_station2
{
// 构造函数，初始化变量
    GroundStation2::GroundStation2(QWidget* parent): rviz::Panel(parent)
    {
        // 水平布局
        QVBoxLayout* layout = new QVBoxLayout;

        // 垂直布局
        QVBoxLayout* vlayout = new QVBoxLayout;
        layout -> addLayout(vlayout);

        // 状态列表
        _uavStatusTable = new QTableWidget;
        vlayout->addWidget(_uavStatusTable);

        // 重置按钮
        _reset = new QPushButton("重置");
        vlayout -> addWidget(_reset);

        // 开始按钮
        _start = new QPushButton("开始");
        vlayout -> addWidget(_start);

        // 输入框
        _lineEdit = new QLineEdit;
        vlayout->addWidget(_lineEdit);
        //让输入框显示“一键清除”按钮
        _lineEdit->setClearButtonEnabled(true);

//        _lineEdit->textChanged();

        // 设置表格的行
        _uavStatusTable->setRowCount(3);
        // 设置表格的列
        _uavStatusTable->setColumnCount(4);
        // 设置表格中每一行的表头
        _uavStatusTable->setHorizontalHeaderLabels(QStringList() << "编号" << "GPS" << "高度(m)" << "电机");
        // 表列随着表格变化而自适应变化
        _uavStatusTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        // 表行随着表格变化而自适应变化
        // _uavStatusTable->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        /* 另一种设置表格表头的方法
        QStringList table_header;
        table_header << "编号" << "GPS" << "高度(m)" << "电机";
        _uavStatusTable->setHorizontalHeaderLabels(table_header);
        */
        for(int i(0); i<4; i++){
            _uavStatusTable->horizontalHeaderItem(i)->setTextAlignment(Qt::AlignLeft);
        }
        // 设置表格数据区内的所有单元格都不允许编辑
        _uavStatusTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        // 设置表格中每一行的内容
        _uavStatusTable->setItem(0,0,new QTableWidgetItem("uav1", Qt::AlignHCenter));
        _uavStatusTable->setItem(0,1,new QTableWidgetItem("▮▮▮▮▮"));
        _uavStatusTable->setItem(0,2,new QTableWidgetItem("13.7979"));
        _uavStatusTable->setItem(0,3,new QTableWidgetItem("on"));
        _uavStatusTable->setItem(1,0,new QTableWidgetItem("uav2"));
        _uavStatusTable->setItem(1,1,new QTableWidgetItem("▮▮▮▮▮"));
        _uavStatusTable->setItem(1,2,new QTableWidgetItem("13.7979"));
        _uavStatusTable->setItem(1,3,new QTableWidgetItem("on"));
        // center();
        _uavStatusTable->show();

        setLayout(layout);

        // 设置信号与槽的连接
        // 和标准的 Qt 程序不同，rviz 的插件编写的信号声明可以直接填写信号函数，如，SIGNAL(clicked())，
        // 而不能和标准 Qt 程序用一样形式，即在函数前加控件 SIGNAL(_reset->clicked()) , 这样会导致信号无法识别
        connect(_uavStatusTable, SIGNAL(itemClicked(QTableWidgetItem *)), this, SLOT(updateStatus()));
        connect(_reset, SIGNAL(clicked()), this, SLOT(updateStatus()));
        connect(_start, SIGNAL(clicked()), this, SLOT(start()));
        connect(_lineEdit, SIGNAL(textEdited(const QString &)),this, SLOT(abc()));
    }


    // 点击表格会刷新 & 重置按钮
    void GroundStation2::updateStatus(){
        _uavStatusTable->clearContents();
        _uavStatusTable->setRowCount(3);
        _uavStatusTable->setColumnCount(4);
    }

    // start 按钮的功能
    void GroundStation2::start(){
        _uavStatusTable->clearContents();
        _uavStatusTable->setRowCount(3);
        _uavStatusTable->setColumnCount(4);
        QTableWidgetItem abc("uav11", 4);
        abc.setText("uav11");
        abc.setTextAlignment(Qt::AlignHCenter);

        QTableWidgetItem *aaa = &abc;
        // 设置表格中每一行的内容
        _uavStatusTable->setItem(0,0,new QTableWidgetItem("uav12"));
        _uavStatusTable->setItem(0,1,new QTableWidgetItem("▮▮▮▮▮"));
        _uavStatusTable->setItem(0,2,new QTableWidgetItem("13.7979"));
        _uavStatusTable->setItem(0,3,new QTableWidgetItem("on"));
        _uavStatusTable->setItem(1,0,new QTableWidgetItem("uav9"));
        _uavStatusTable->setItem(1,1,new QTableWidgetItem("▮▮▮▮▮"));
        _uavStatusTable->setItem(1,2,new QTableWidgetItem("13.7979"));
        _uavStatusTable->setItem(1,3,new QTableWidgetItem("on"));
        // center();
        _uavStatusTable->show();
    }

    // 居中表格中的文本
    void GroundStation2::center(){
        for(int i(0); i < 2; i++){
            for(int j(0); j < 4; j++){
                _uavStatusTable->item(i, j)->setTextAlignment(Qt::AlignHCenter);
            }
        }
    }

    void GroundStation2::abc(){
        _uavStatusTable->setItem(0, 0, new QTableWidgetItem(_lineEdit->text()));
        _lineEdit->clear();
    }

} // end namespace rviz_plugin_tutorials

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ground_control_station2::GroundStation2,rviz::Panel )
