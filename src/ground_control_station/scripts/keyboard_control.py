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
 |        3 : Switch single mode         |
 |        4 : Round up                   |
 |        5 : Correcte height            |
 |        6 : input id
 |        Tab: Switch frame              |     
 |                                       | 
 =========================================

CTRL-C to quit

"""
single_mode = False
zhaji = False
zhaji_id = '0'
cmd = 'illegal'
uavName = 'uav1'


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


def switch_single_mode(single_mode):
    if single_mode:
        print('\n============ Enter a single mode ============')
        print('\nPlease input number of quadrotors(eg. 1#) :')
        cut_mode = False
        timeout = 1000
        uavNumber = get_key(cut_mode, timeout)
        # 字符串切片，截取uavNumber开头到字符'#'的字符（去除回车符号）
        uavName = 'uav' + uavNumber[:uavNumber.find('#') + 1]
        print('\n=============================================\n')
        return uavName
    else:
        print('\n============ Enter group mode ============\n')

def input_zhaji():
    global zhaji
    if zhaji:
        print('\n============ Enter debug mode ============')
        print('\nPlease input number of quadrotors(eg. 33) :')
        cut_mode = False
        timeout = 1000
        id_tmp = get_key(cut_mode, timeout)
        id = id_tmp.replace('\r', '').replace('\n', '').replace('\t', '')
        # 字符串切片，截取uavNumber开头到字符'#'的字符（去除回车符号）
        # uavName = 'uav' + uavNumber[:uavNumber.find('#') + 1]
        zhaji = False
        print("id = ", id)
        id_pub = rospy.Publisher('/input_id', String, queue_size=1)
        id_pub.publish(id)
        print('\n============ End debug mode ============\n')
        return id

def tasks_publish(pub, key):
    global single_mode
    global zhaji
    global cmd
    global uavName
    if key == '0':
        cmd = 'MotorUnlock'
    elif key == '1':
        cmd = 'Takeoff'
    elif key == '2':
        cmd = 'Land'
    elif key == '3':
        single_mode = bool(1 - single_mode)
        uavName = switch_single_mode(single_mode)
    elif key == '4':
        cmd = 'RoundUp'
    elif key == '5':
        cmd = 'CorrecteHeight'
    elif key == '6':
        zhaji = bool(1 - zhaji)
        zhaji_id = input_zhaji()
        print("id = ", zhaji_id)
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
    elif key == 'v':
        cmd = 'v'
    elif key == 'b':
        cmd = 'b'
    # elif key == '4':
    #     cmd = 'SetLocalFrame'
    else:
        cmd = 'illegal'
    if cmd != 'illegal':
        if single_mode:
            msg = String()
            cmd = uavName + cmd
            msg.data = cmd
            pub.publish(msg)
            print('Send command ' + cmd + ' successfully.')
        else:
            msg = String()
            msg.data = cmd
            pub.publish(msg)
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
