#!/usr/bin/env python
# coding=utf-8
import rospy
import sys, select, tty, termios
from std_msgs.msg import String


msg = """
 ============= action control ============
 |                                       | 
 |                   w                   | 
 |              a    s    d              | 
 |                                       | 
 |         w and s : pitch control       |
 |         a and d : roll control        | 
 |                                       | 
 |---------------------------------------|
 |                                       | 
 |                   i                   |
 |              j    k    l              |
 |                                       |     
 |        i and k : throttle control     | 
 |        j and l : yaw control          | 
 |                                       | 
 |---------------------------------------|
 |                                       | 
 |        g : Speed increase             | 
 |        h : Speed reduction            |     
 |        space: Stop                    | 
 |                                       | 
 |============ command control ===========
 |                                       | 
 |        0 : Motor unlock               | 
 |        1 : Takeoff                    | 
 |        2 : Land                       | 
 |        3 : Swarm control              |
 |        4 : Go Home                    |
 |        Tab: Switch frame              |     
 |                                       | 
 =========================================

CTRL-C to quit

"""


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
    if key == '0':
        cmd = 'MotorUnlock'
    elif key == '1':
        cmd = 'Takeoff'
    elif key == '2':
        cmd = 'Land'
    elif key == '3':
        cmd = 'SwarmControl'
    elif key == '4':
        cmd = 'GoHome'
    elif key == 'w':
        cmd = 'Forward'
    elif key == 's':
        cmd = 'Backward'
    elif key == 'a':
        cmd = 'TurnLeft'
    elif key == 'd':
        cmd = 'TurnRight'
    elif key == 'i':
        cmd = 'Upward'
    elif key == 'k':
        cmd = 'Down'
    elif key == 'j':
        cmd = 'RotateLeft'
    elif key == 'l':
        cmd = 'RotateRight'
    elif key == 'g':
        cmd = 'SpeedUp'
    elif key == 'h':
        cmd = 'SpeedDown'
    elif key == ' ':
        cmd = 'Stop'
    elif key == '\x09':
        cmd = 'switchFrame'
    else:
        cmd = 'illegal'
    if cmd != 'illegal':
        if cmd != 'SwarmControl':
            msg = String()
            msg.data = cmd
            pub.publish(msg)
            print('Send command ' + cmd + ' successfully.')
        else:
            print('\nPlease input num of quadrotors:')
            cut_mode = False
            timeout = 1000
            num = int(get_key(cut_mode, timeout))
            rospy.set_param('/uavNumbers', num)
            pub.publish(cmd)
            print('Send command ' + cmd + ' successfully.')


if __name__ == '__main__':
    rospy.init_node('keyboard_control', anonymous=True)
    key_pub = rospy.Publisher('key', String, queue_size=1)
    rate = rospy.Rate(100)  # 100Hz
    print(msg)

    # 备份终端属性
    old_attr = termios.tcgetattr(sys.stdin)

    while not rospy.is_shutdown():
        # 获取终端键值
        cut_mode = True
        timeout = 0.1
        key = get_key(cut_mode, timeout)
        # 发布键值
        tasks_publish(key_pub, key)
        # ctrl+c ascll 编码为 \003
        if key == '\003':
            break
        rate.sleep()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
