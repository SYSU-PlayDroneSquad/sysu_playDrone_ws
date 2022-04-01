#!/usr/bin/env python
# coding=utf-8

# import ros files
import rospy
from geometry_msgs.msg import Twist

# import other files
from fifty_48AR_sim import ST8
from environment import Env
from ground_control_station.msg import Array3
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
import numpy as np

hunt_end = True
in_list = np.transpose(np.zeros(48))
gap_error = 0
g_seq = []
# g_seq = [8, 10, 9, 11, 13, 12, 14, 16, 15, 17, 19, 18]
target_vel = [0, 0]
target_pos = [0, 0]

# 输出控制速度到屏幕
def output_screen(vel_xy, seqq):
    print("Whether hunt end:")
    print(hunt_end)
    print("Agents in range:")
    print(in_list)
    print("gap error")
    print(gap_error)
    #print("Secquence")
    #print(seqq)


# 追捕
def hunt(p_locs, e_loc, p, e_v):
    env1 = Env(256, 256)  # 实例化 Env 类
    env1.input(p_locs, e_loc, p, e_v)
    global hunt_end
    global in_list
    global gap_error
    global seq
    vel_xy, hunt_end, in_list, gap_error, seq = env1.run()
    output_screen(vel_xy, seq)
    return vel_xy, seq


# 围猎
st8 = ST8()
def treibjagd(pos_arr, txy, seq, target_vel):
    vel_xy, land_cmd, attack_cmd = st8.op_vol(pos_arr, txy, seq, target_vel)
    # output_screen(vel_xy)
    return vel_xy, land_cmd, attack_cmd

def pub_vel(uavNum, vel_xy):
    vel_list = Array3()
    vel_pub_test = rospy.Publisher('/vel_ll', Array3, queue_size=10)
    for i in range(uavNum):
        vel_list.x.append(vel_xy[0, i])
        vel_list.y.append(vel_xy[1, i])
        vel_list.z.append(vel_xy[2, i])
    vel_pub_test.publish(vel_list)

def target_sub_CB(model_states):
    global target_vel
    global target_pos
    target_pos[0] = model_states.pose[1].position.x
    target_pos[1] = model_states.pose[1].position.y
    target_vel[0] = model_states.twist[1].linear.x
    target_vel[1] = model_states.twist[1].linear.x


def pos_sub_CB1(pos, pub_list):
    uavNum = 24
    p_locs = []  # -------------------------- 无人机位置
    pos_arr = np.zeros((3, uavNum), dtype='f8')  # 无人机位置
    global g_seq
    global target_vel
    global target_pos
    print("target_pos = ", target_pos)
    for i in range(uavNum):
        pos_arr[0, i] = pos.x[i]
        pos_arr[1, i] = pos.y[i]
        pos_arr[2, i] = pos.z[i]
        loc = np.array(([pos.x[i], pos.y[i]]))
        p_locs.append(loc)

    # ============================ 追捕或围猎 ============================
    vel_xy = np.zeros((3, uavNum), dtype='f8')  # 速度控制
    v_xy = np.zeros((2, uavNum), dtype='f8')
    e_loc = [0, 0.1]
    #e_loc = [0, 0.1]

    if not hunt_end:

        v_xy, seq = hunt(p_locs, target_pos, in_list, target_vel)
        g_seq = seq

        #print g_seq
    else:

        # vel_xy = treibjagd(pos_arr, e_loc, g_seq)
        vel_xy, land_cmd, attack_cmd = treibjagd(pos_arr, target_pos, g_seq, target_vel)

        cmd_msg = String()
        cmd_msg.data = land_cmd
        key_pub.publish(cmd_msg)
        # pub_vel(uavNum, vel_xy)


    # =============================  发布  =============================
    twist_msg = Twist()
    for i in range(uavNum):
        if not hunt_end:
            # print("追捕中...")
            twist_msg.linear.x = v_xy[0, i]
            twist_msg.linear.y = v_xy[1, i]
        else:
            # print("围猎中...")
            twist_msg.linear.x = vel_xy[0, i]
            twist_msg.linear.y = vel_xy[1, i]
            twist_msg.linear.z = vel_xy[2, i]
        pub_list[i].publish(twist_msg)
        # print(twist_msg)


if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)
    vel_pub1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
    vel_pub2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=1)
    vel_pub3 = rospy.Publisher('/uav3/cmd_vel', Twist, queue_size=1)
    vel_pub4 = rospy.Publisher('/uav4/cmd_vel', Twist, queue_size=1)
    vel_pub5 = rospy.Publisher('/uav5/cmd_vel', Twist, queue_size=1)
    vel_pub6 = rospy.Publisher('/uav6/cmd_vel', Twist, queue_size=1)
    vel_pub7 = rospy.Publisher('/uav7/cmd_vel', Twist, queue_size=1)
    vel_pub8 = rospy.Publisher('/uav8/cmd_vel', Twist, queue_size=1)
    vel_pub9 = rospy.Publisher('/uav9/cmd_vel', Twist, queue_size=1)
    vel_pub10 = rospy.Publisher('/uav10/cmd_vel', Twist, queue_size=1)
    vel_pub11 = rospy.Publisher('/uav11/cmd_vel', Twist, queue_size=1)
    vel_pub12 = rospy.Publisher('/uav12/cmd_vel', Twist, queue_size=1)
    vel_pub13 = rospy.Publisher('/uav13/cmd_vel', Twist, queue_size=1)
    vel_pub14 = rospy.Publisher('/uav14/cmd_vel', Twist, queue_size=1)
    vel_pub15 = rospy.Publisher('/uav15/cmd_vel', Twist, queue_size=1)
    vel_pub16 = rospy.Publisher('/uav16/cmd_vel', Twist, queue_size=1)
    vel_pub17 = rospy.Publisher('/uav17/cmd_vel', Twist, queue_size=1)
    vel_pub18 = rospy.Publisher('/uav18/cmd_vel', Twist, queue_size=1)
    vel_pub19 = rospy.Publisher('/uav19/cmd_vel', Twist, queue_size=1)
    vel_pub20 = rospy.Publisher('/uav20/cmd_vel', Twist, queue_size=1)
    vel_pub21 = rospy.Publisher('/uav21/cmd_vel', Twist, queue_size=1)
    vel_pub22 = rospy.Publisher('/uav22/cmd_vel', Twist, queue_size=1)
    vel_pub23 = rospy.Publisher('/uav23/cmd_vel', Twist, queue_size=1)
    vel_pub24 = rospy.Publisher('/uav24/cmd_vel', Twist, queue_size=1)
    vel_pub25 = rospy.Publisher('/uav25/cmd_vel', Twist, queue_size=1)
    vel_pub26 = rospy.Publisher('/uav26/cmd_vel', Twist, queue_size=1)
    vel_pub27 = rospy.Publisher('/uav27/cmd_vel', Twist, queue_size=1)
    vel_pub28 = rospy.Publisher('/uav28/cmd_vel', Twist, queue_size=1)
    vel_pub29 = rospy.Publisher('/uav29/cmd_vel', Twist, queue_size=1)
    vel_pub30 = rospy.Publisher('/uav30/cmd_vel', Twist, queue_size=1)
    vel_pub31 = rospy.Publisher('/uav31/cmd_vel', Twist, queue_size=1)
    vel_pub32 = rospy.Publisher('/uav32/cmd_vel', Twist, queue_size=1)
    vel_pub33 = rospy.Publisher('/uav33/cmd_vel', Twist, queue_size=1)
    vel_pub34 = rospy.Publisher('/uav34/cmd_vel', Twist, queue_size=1)
    vel_pub35 = rospy.Publisher('/uav35/cmd_vel', Twist, queue_size=1)
    vel_pub36 = rospy.Publisher('/uav36/cmd_vel', Twist, queue_size=1)
    vel_pub37 = rospy.Publisher('/uav37/cmd_vel', Twist, queue_size=1)
    vel_pub38 = rospy.Publisher('/uav38/cmd_vel', Twist, queue_size=1)
    vel_pub39 = rospy.Publisher('/uav39/cmd_vel', Twist, queue_size=1)
    vel_pub40 = rospy.Publisher('/uav40/cmd_vel', Twist, queue_size=1)
    vel_pub41 = rospy.Publisher('/uav41/cmd_vel', Twist, queue_size=1)
    vel_pub42 = rospy.Publisher('/uav42/cmd_vel', Twist, queue_size=1)
    vel_pub43 = rospy.Publisher('/uav43/cmd_vel', Twist, queue_size=1)
    vel_pub44 = rospy.Publisher('/uav44/cmd_vel', Twist, queue_size=1)
    vel_pub45 = rospy.Publisher('/uav45/cmd_vel', Twist, queue_size=1)
    vel_pub46 = rospy.Publisher('/uav46/cmd_vel', Twist, queue_size=1)
    vel_pub47 = rospy.Publisher('/uav47/cmd_vel', Twist, queue_size=1)
    vel_pub48 = rospy.Publisher('/uav48/cmd_vel', Twist, queue_size=1)


    # pub_list = [vel_pub1, vel_pub2, vel_pub3, vel_pub4, vel_pub5, vel_pub6, vel_pub7, vel_pub8]
    pub_list = [vel_pub1, vel_pub2, vel_pub3, vel_pub4, vel_pub5, vel_pub6, vel_pub7, vel_pub8,
                vel_pub9, vel_pub10, vel_pub11, vel_pub12, vel_pub13, vel_pub14, vel_pub15, vel_pub16,
                vel_pub17, vel_pub18, vel_pub19, vel_pub20, vel_pub21, vel_pub22, vel_pub23, vel_pub24,
                vel_pub25, vel_pub26, vel_pub27, vel_pub28, vel_pub29, vel_pub30, vel_pub31, vel_pub32,
                vel_pub33, vel_pub34, vel_pub35, vel_pub36, vel_pub37, vel_pub38, vel_pub39, vel_pub40,
                vel_pub41, vel_pub42, vel_pub43, vel_pub44, vel_pub45, vel_pub46, vel_pub47, vel_pub48]
                
    key_pub = rospy.Publisher('/key', String, queue_size=1)
    target_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, target_sub_CB)

    pos_sub = rospy.Subscriber('/pos_arr', Array3, pos_sub_CB1, pub_list)

    rospy.spin()
