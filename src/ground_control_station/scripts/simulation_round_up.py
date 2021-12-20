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

hunt_end = True


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
    env1 = Env(256, 256)  # 实例化 Env 类
    env1.input(p_locs, e_loc)
    vel_xy = env1.run()
    output_screen(vel_xy)
    return vel_xy


# 围猎
st8 = ST8()
def treibjagd(pos_arr):
    global st8
    vel_xy = st8.op_vol(pos_arr)
    output_screen(vel_xy)
    return vel_xy


def pos_sub_CB1(pos, pub_list):
    p_locs = []  # -------------------------- 无人机位置
    pos_arr = np.zeros((3, 8), dtype='f8')  # 无人机位置

    for i in range(8):
        pos_arr[0, i] = pos.x[i]
        pos_arr[1, i] = pos.y[i]
        pos_arr[2, i] = pos.z[i]
        loc = np.array(([pos.x[i], pos.y[i]]))
        p_locs.append(loc)

    # ============================ 追捕或围猎 ============================
    vel_xy = np.zeros((3, 8), dtype='f8')  # 速度控制
    v_xy = np.zeros((2, 8), dtype='f8')
    e_loc = [0, 24]  # 目标点位置
    if not hunt_end:
        v_xy = hunt(p_locs, e_loc)
    else:
        vel_xy = treibjagd(pos_arr)

    # =============================  发布  =============================
    twist_msg = Twist()
    for i in range(8):
        if not hunt_end:
            twist_msg.linear.x = v_xy[0, i]
            twist_msg.linear.y = v_xy[1, i]
        else:
            twist_msg.linear.x = vel_xy[0, i]
            twist_msg.linear.y = vel_xy[1, i]
            twist_msg.linear.z = vel_xy[2, i]
        pub_list[i].publish(twist_msg)
        # print(twist_msg)


if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    vel_pub1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
    vel_pub2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=1)
    vel_pub3 = rospy.Publisher('/uav3/cmd_vel', Twist, queue_size=1)
    vel_pub4 = rospy.Publisher('/uav4/cmd_vel', Twist, queue_size=1)
    vel_pub5 = rospy.Publisher('/uav5/cmd_vel', Twist, queue_size=1)
    vel_pub6 = rospy.Publisher('/uav6/cmd_vel', Twist, queue_size=1)
    vel_pub7 = rospy.Publisher('/uav7/cmd_vel', Twist, queue_size=1)
    vel_pub8 = rospy.Publisher('/uav8/cmd_vel', Twist, queue_size=1)

    pub_list = [vel_pub1, vel_pub2, vel_pub3, vel_pub4, vel_pub5, vel_pub6, vel_pub7, vel_pub8]

    pos_sub = rospy.Subscriber('/pos_arr', Array3, pos_sub_CB1, pub_list)

    abc = [1, 2, 3, 4]

    rospy.spin()
