#!/usr/bin/env python
# coding=utf-8
import rospy
import sys, select, tty, termios
from std_msgs.msg import String
from ground_control_station.msg import Array3

cmd = '0'
vel_arr = Array3()
for i in range(60):
    vel_arr.x.append(0.0)
    vel_arr.y.append(0.0)
    vel_arr.z.append(0.0)

def get_key(cut_mode, timeout):
    if cut_mode:
        # 更改终端属性为直接读取不显示
        tty.setraw(sys.stdin.fileno())

    # 阻塞 timeout 秒, 如果输入流有数据则 rlist 为 1 ，否则为 0
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        if cut_mode:
            # 读取终端的 1 个字符后就返回
            key = sys.stdin.read(1)
        else:
            # 读取终端的 1 行字符后就返回
            key = sys.stdin.readline()
    else:
        key = ''

    # 将终端设置会原来的标准属性 old_attr
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    return key

def tasks_publish(pub, key):
    global cmd
    global vel_arr
    if key == '0':
        cmd = '0'
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = 0.0
    elif key == '1':
        cmd = '1'
    elif key == '2':
        cmd = '2'
    elif key == '3':
        cmd = '3'
    elif key == '4':
        cmd = '4'
    elif key == 'w':
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = int(cmd)
            vel_arr.z[i] = 0.0
    elif key == 'd':
        for i in range(60):
            vel_arr.x[i] = int(cmd)
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = 0.0
    elif key == 'a':
        for i in range(60):
            vel_arr.x[i] = -int(cmd)
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = 0.0
    elif key == 's':
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = -int(cmd)
            vel_arr.z[i] = 0.0
    elif key == 'i':
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = int(cmd)
    elif key == 'j':
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = -1.0
    elif key == ' ':
        for i in range(60):
            vel_arr.x[i] = 0.0
            vel_arr.y[i] = 0.0
            vel_arr.z[i] = 0.0
    pub.publish(vel_arr)
    print('Send command ' + cmd + ' m/s' + ' successfully.')

if __name__ == '__main__':
    rospy.init_node('forward_flight', anonymous=True)
    vel_pub = rospy.Publisher('/velocity_list', Array3, queue_size=1)
    rate = rospy.Rate(50)  # 50Hz

    # 备份终端属性
    old_attr = termios.tcgetattr(sys.stdin)

    while not rospy.is_shutdown():
        # 获取终端键值
        cut_mode = True
        timeout = 0.1
        key = get_key(cut_mode, timeout)
        # print (key)
        # 发布键值
        tasks_publish(vel_pub, key)
        # ctrl+c ascll 编码为 \003
        if key == '\003':
            break
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
