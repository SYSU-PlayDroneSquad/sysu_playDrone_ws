#!/usr/bin/env python
# coding=utf-8

import rospy
from gazebo_msgs.msg import ModelStates

def vel_cb(modeStates):

    print 'uav1', ':', modeStates.twist[11].linear.x, '  ', modeStates.twist[11].linear.y, '  ', modeStates.twist[1].linear.z
    print 'uav3', ':', modeStates.twist[23].linear.x, '  ', modeStates.twist[23].linear.y, '  ', modeStates.twist[23].linear.z
    print 'uav5', ':', modeStates.twist[10].linear.x, '  ', modeStates.twist[10].linear.y, '  ', modeStates.twist[10].linear.z
    print 'uav7', ':', modeStates.twist[19].linear.x, '  ', modeStates.twist[19].linear.y, '  ', modeStates.twist[19].linear.z
    print ''
if __name__ == '__main__':
    rospy.init_node('ehcho_vel', anonymous=True)
    vel_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, vel_cb)
    rospy.spin()