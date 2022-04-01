#!/usr/bin/env python
# coding=utf-8

import rospy
from ground_control_station.msg import Array3
from std_msgs.msg import Int32MultiArray

from std_msgs.msg import String
import numpy as np

id_list = Int32MultiArray()


def key_cb(msg):
    global id_list
    if msg.data == 'GetIdList':
        print('get id list')
        get_id_list()
    elif msg.data == 'Track':
        print ""
        print('================== track flight ==================')
        track()


def get_id_list():
    global id_list
    msg = rospy.wait_for_message("/id_list", Int32MultiArray, timeout=3)
    id_list = msg
    print "[data]:", msg.data


def output_screen(seq, step):
    seq[0] += 1
    n = seq[0] / 3.0
    if n == 1.0:
        if seq[1]:
            print "\033[2A\033[K"
        print step, "....\033[3D\033[K"
    elif n == 2.0:
        # 之所以上移动两行，是因为上一条语句占了一行，
        # 另外print "\033[2A\033[K" 语句本身也会占一行
        print "\033[2A\033[K"
        print step, "....\033[2D\033[K"
    elif n == 3.0:
        print "\033[2A\033[K"
        print step, "....\033[1D\033[K"
    elif n == 4.0:
        print "\033[2A\033[K"
        print step, "...."
        seq[0] = 0
        seq[1] = 1


def track():
    global id_list
    global uavNumbers
    global uavIds
    vel_arr = Array3()

    rate = rospy.Rate(25)
    for i in range(uavIds):
        vel_arr.x.append(0.0)
        vel_arr.y.append(0.0)
        vel_arr.z.append(0.0)
    if len(id_list.data) < uavNumbers:
        print('Not init id_list!')
        return
    finally_front_index = id_list.data[uavNumbers]
    finally_later_index = uavNumbers - 1
    for i in range(uavNumbers):
        if id_list.data[i] == 0:
            finally_later_index = i - 1
            break
    print 'finally_front_index = ', finally_front_index
    print 'finally_later_index = ', finally_later_index
    # dji 的 flu 转 enu 从 x 轴开始，逆时针转 0~180° yaw 值为正，顺时针 0~180° yaw 值为负,
    # 即在 x 轴上方的角是正的，x 轴下方的角是负的
    # djiN3 飞控的 flu 以前向为 x 轴正方向，左为 y 轴正方向, enu 以东为 x 轴正方向， 以北为 y 轴正方向
    # djiN3 飞控传回来的姿态角四元数的 yaw 值，是无人机 x 轴正方向与 大地坐标系的 x 轴正方向(东方)的夹角，同上，夹角在ENU x 轴上方为正，下方为负
    # djiN3 的速度控制接口，即/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate,输入的是大地坐标系的速度

    # ============= step1 ============= ——————————————> 东(x) 北(y)#
    for i in range(finally_later_index + 1):
        if i <= finally_front_index:
            vel_arr.x[id_list.data[i] - 1] = 0.0
            vel_arr.y[id_list.data[i] - 1] = 3.0
            vel_arr.z[id_list.data[i] - 1] = 0.0
        else:
            vel_arr.x[id_list.data[i] - 1] = 0.0
            vel_arr.y[id_list.data[i] - 1] = -3.0
            vel_arr.z[id_list.data[i] - 1] = 0.0

    step1_begin = rospy.get_time()
    seq = [0, 0]
    step = "step1"
    print ""
    while rospy.get_time() - step1_begin < 10:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        output_screen(seq, step)
        rate.sleep()
    # ============= step2 ============= ——————————————> 东(x)  #
    for i in range(finally_later_index + 1):
        if i <= finally_front_index:
            vel_arr.x[id_list.data[i] - 1] = 3.0
            vel_arr.y[id_list.data[i] - 1] = 0.0
            vel_arr.z[id_list.data[i] - 1] = 0.0
        else:
            vel_arr.x[id_list.data[i] - 1] = 3.0
            vel_arr.y[id_list.data[i] - 1] = 0.0
            vel_arr.z[id_list.data[i] - 1] = 0.0

    step2_begin = rospy.get_time()
    seq = [0, 0]
    print "step1...."
    step = "step2"
    while rospy.get_time() - step2_begin < 5:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        output_screen(seq, step)
        rate.sleep()

    # ============= step3 ============= ——————————————> 东(x)  #
    for i in range(finally_later_index + 1):
        if i <= finally_front_index:
            vel_arr.x[id_list.data[i] - 1] = 0.0
            vel_arr.y[id_list.data[i] - 1] = -3.0
            vel_arr.z[id_list.data[i] - 1] = 0.0
        else:
            vel_arr.x[id_list.data[i] - 1] = 0.0
            vel_arr.y[id_list.data[i] - 1] = 3.0
            vel_arr.z[id_list.data[i] - 1] = 0.0

    step3_begin = rospy.get_time()
    seq = [0, 0]
    print "step2...."
    step = "step3"
    while rospy.get_time() - step3_begin < 10:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        output_screen(seq, step)
        rate.sleep()
    print "End track!"

if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)
    key_sub = rospy.Subscriber('/key', String, key_cb)

    uavNumbers = rospy.get_param("/ground_control_station/uavNumbers", 10)
    uavIds = rospy.get_param("/ground_control_station/uavIds", 50)

    print "uavNumbers = ", uavNumbers
    print "uavIds = ", uavIds

    rospy.spin()
