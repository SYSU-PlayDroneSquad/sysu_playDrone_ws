#!/usr/bin/env python
# coding=utf-8

import rospy
from ground_control_station.msg import Array3
import numpy as np


if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)

    # ============= step1 ============= ——————————————> 东(x) #
    vel_arr = Array3()
    for i in range(22):
        vel_arr.x.append(0.0)
        vel_arr.y.append(2.0)
        vel_arr.z.append(0.0)

    step1_begin = rospy.get_time()
    rate = rospy.Rate(10)
    # print 'step1' ,  step1_begin
    while rospy.get_time() - step1_begin < 10:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        rate.sleep()

    # ============= step2 ============= ——————————————> 东(x)  #
    for i in range(22):
        vel_arr.x[i] = 2.0
        vel_arr.y[i] = 0.0
        vel_arr.z[i] = 0.0

    step2_begin = rospy.get_time()
    while rospy.get_time() - step2_begin < 5:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        rate.sleep()

    # ============= step3 ============= ——————————————> 东(x)  #
    for i in range(22):
        vel_arr.x[i] = 0.0
        vel_arr.y[i] = -2.0
        vel_arr.z[i] = 0.0

    step3_begin = rospy.get_time()
    while rospy.get_time() - step2_begin < 10:
        if rospy.is_shutdown():
            break
        vel_pub.publish(vel_arr)
        rate.sleep()