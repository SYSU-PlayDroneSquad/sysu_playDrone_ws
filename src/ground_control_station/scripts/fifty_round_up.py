#!/usr/bin/env python
# coding=utf-8

# import ros files
import rospy
from geometry_msgs.msg import Twist

# import other files
from fifty_8test import ST8
from environment import Env
from ground_control_station.msg import Array3
import numpy as np


# 输出控制速度到屏幕
def output_screen(vel_xy):
    for i in range(8):
        print('x' + str(i) + ' = ' + str(vel_xy[0][i])
              + '  y' + str(i) + ' = ' + str(vel_xy[1][i])
              + '  z' + str(i) + ' = ' + str(vel_xy[2][i]))
    print("")


hunt_end = False


# 追捕
def hunt(p_locs, e_loc):
    env1 = Env(256, 256)  # 实例化 Env 类
    env1.input(p_locs, e_loc)
    global hunt_end
    vel_xy, hunt_end = env1.run()
    # output_screen(vel_xy)
    return vel_xy


# 围猎
def treibjagd(pos_arr):
    st8 = ST8()
    vel_xy = st8.op_vol(pos_arr)
    # output_screen(vel_xy)
    return vel_xy


# 位置订阅回调
def pos_sub_cb(pos, vel_pub, uavNumbers):
    p_locs = []  # -------------------------- 无人机位置
    pos_arr = np.zeros((3, uavNumbers), dtype='f8')  # 无人机位置

    for i in range(uavNumbers):
        pos_arr[0, i] = pos.x[i]
        pos_arr[1, i] = pos.y[i]
        pos_arr[2, i] = pos.z[i]
        loc = np.array(([pos.x[i], pos.y[i]]))
        p_locs.append(loc)

    # ============================ 追捕或围猎 ============================
    vel_xy = np.zeros((3, uavNumbers), dtype='f8')  # 速度控制
    v_xy = np.zeros((2, uavNumbers), dtype='f8')
    e_loc = [35.11905, 54.52596]  # 目标点位置
    if not hunt_end:
        v_xy = hunt(p_locs, e_loc)
    else:
        vel_xy = treibjagd(pos_arr)

    # =============================  发布  =============================
    vel_arr = Array3()
    vel_arr.y.insert()
    for i in range(uavNumbers):
        if not hunt_end:
            vel_arr.x[i] = v_xy[0, i]
            vel_arr.y[i] = v_xy[1, i]
            vel_arr.z.append(0)
        else:
            vel_arr[0, i] = vel_xy[0, i]
            vel_arr[1, i] = vel_xy[1, i]
            vel_arr[2, i] = vel_xy[2, i]

    vel_pub.publish(vel_arr)


if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    uavNumbers = 8
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)
    pos_sub = rospy.Subscriber('/position_list', Array3, pos_sub_cb, vel_pub, uavNumbers)
    rospy.spin()
