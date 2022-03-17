#!/usr/bin/env python
# coding=utf-8

import rospy
from ground_control_station.msg import StatusArrayNew
from ground_control_station.msg import StatusNew
from ground_control_station.msg import StatusArray

def status_sub_cb(status, status_pub):
    statusArrayNew = StatusArrayNew()
    for i in range(8):
        statusNew = StatusNew()
        statusNew.header = status.StatusArray[i].header
        statusNew.id = status.StatusArray[i].id
        statusNew.seq = status.StatusArray[i].seq
        statusNew.lv_gps = status.StatusArray[i].lv_gps
        statusNew.flight_status = 1
        statusNew.latitude = status.StatusArray[i].latitude
        statusNew.longitude = status.StatusArray[i].longitude
        statusNew.altitude = status.StatusArray[i].altitude
        statusNew.quaternion = status.StatusArray[i].quaternion
        statusArrayNew.StatusArray.append(statusNew)

    status_pub.publish(statusArrayNew)

if __name__ == '__main__':
    rospy.init_node('relay_msg', anonymous=True)
    status_pub = rospy.Publisher('/abc', StatusArrayNew, queue_size=1)
    status_sub = rospy.Subscriber('/UAVs/status', StatusArray, status_sub_cb, status_pub)
    rospy.spin()