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

hunt_end = False


# 输出控制速度到屏幕
def output_screen(vel_xy):
    for i in range(8):
        print('x' + str(i) + ' = ' + str(vel_xy[0][i])
              + '  y' + str(i) + ' = ' + str(vel_xy[1][i])
              + '  z' + str(i) + ' = ' + str(vel_xy[2][i]))
    print("")


# 追捕状态
def hunt_status(status):
    global hunt_end
    hunt_end = status


# 追捕
def hunt(p_locs, e_loc):
    env1 = Env(256, 256)    # 实例化 Env 类
    env1.input(p_locs, e_loc)
    vel_xy = env1.run()
    # output_screen(vel_xy)
    return vel_xy


# 围猎
def treibjagd(pos_arr):
    st8 = ST8()
    vel_xy = st8.op_vol(pos_arr)
    # output_screen(vel_xy)
    return vel_xy


# 位置订阅回调
def pos_sub_CB1(pos, vel_pub):

    p_locs = []  # -------------------------- 无人机位置
    pos_arr = np.zeros((3, 8), dtype='f8')  # 无人机位置

    for i in range(1):
        pos_arr[0, i] = pos.x[i]
        pos_arr[1, i] = pos.y[i]
        pos_arr[2, i] = pos.z[i]
        loc = np.array(([pos.x[i], pos.y[i]]))
        p_locs.append(loc)

    # ============================ 追捕或围猎 ============================
    vel_xy = np.zeros((3, 8), dtype='f8')   # 速度控制
    v_xy = np.zeros((2, 8), dtype='f8')
    e_loc = [0, 24]  # 目标点位置
    if not hunt_end:
        v_xy = hunt(p_locs, e_loc)
    else:
        vel_xy = treibjagd(pos_arr)

    # =============================  发布  =============================
    # print(v_xy[0][0][0])
    vel_arr = Array3()
    for i in range(8):
        if hunt_end:
            vel_arr.x[i].insert(vel_xy[0, i])
            vel_arr.y[i].insert(vel_xy[1, i])
            vel_arr.z[i].insert(vel_xy[2, i])
        else:
            vel_arr.x.append(v_xy[0][0][i])
            vel_arr.y.append(v_xy[0][1][i])
            vel_arr.z.append(0)
    vel_pub.publish(vel_arr)


if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)
    pos_sub = rospy.Subscriber('/position_list', Array3, pos_sub_CB1, vel_pub)
    rospy.spin()
