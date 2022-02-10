#!/usr/bin/env python
# coding=utf-8

# import ros files
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

# import other files
import numpy as np


def start_point():
    lat1 = 23.0659758791
    lon1 = 113.383029958
    alt1 = -81.4800109863

    lat2 = 23.0659758783
    lon2 = 113.38302998
    alt2 = -81.480255127

    lat3 = 23.0659758823
    lon3 = 113.383029868
    alt3 = -81.4872055054

    lat4 = 23.0659758866
    lon4 = 113.383029689
    alt4 = -81.5026473999

    lat5 = 23.0659759198
    lon5 = 113.383029671
    alt5 = -81.5107955933

    lat6 = 23.0659760057
    lon6 = 113.383029477
    alt6 = -81.4890365601

    lat7 = 23.0659760093
    lon7 = 113.383029436
    alt7 = -81.4921417236

    lat8 = 23.065976008
    lon8 = 113.38302938
    alt8 = -81.4988098145

    lat9 = 23.065975965
    lon9 = 113.383029354
    alt9 = -81.5028305054

    lat10 = 23.0659758977
    lon10 = 113.383029326
    alt10 = -81.5005493164

    latitude = (lat1 + lat2 + lat3 + lat4 + lat5 + lat6 + lat7 + lat8 + lat9 + lat10) / 10
    longitude = (lon1 + lon2 + lon3 + lon4 + lon5 + lon6 + lon7 + lon8 + lon9 + lon10) / 10
    altitude = (alt1 + alt2 + alt3 + alt4 + alt5 + alt6 + alt7 + alt8 + alt9 + alt10) / 10
    print "start point: "
    print (latitude)  # 23.0659759332
    print (longitude)  # 113.383029614
    print (altitude)  # -81.4944282532


def goal_point():
    lat1 = 23.0664712193
    lon1 = 113.383369785
    alt1 = -81.5818939209

    lat2 = 23.0664712155
    lon2 = 113.383369804
    alt2 = -81.5944595337

    lat3 = 23.066471228
    lon3 = 113.38336988
    alt3 = -81.5903091431

    lat4 = 23.0664712025
    lon4 = 113.383369908
    alt4 = -81.5869369507

    lat5 = 23.0664709744
    lon5 = 113.383369816
    alt5 = -81.5517807007

    lat6 = 23.0664709142
    lon6 = 113.383369824
    alt6 = -81.5402297974

    lat7 = 23.0664708575
    lon7 = 113.38336993
    alt7 = -81.58568573

    lat8 = 23.0664708068
    lon8 = 113.383370025
    alt8 = -81.6086044312

    lat9 = 23.0664708041
    lon9 = 113.383370064
    alt9 = -81.6041183472

    lat10 = 23.0664707853
    lon10 = 113.383370084
    alt10 = -81.6085739136
    print "\ngoal point: "
    latitude = (lat1 + lat2 + lat3 + lat4 + lat5 + lat6 + lat7 + lat8 + lat9 + lat10) / 10
    longitude = (lon1 + lon2 + lon3 + lon4 + lon5 + lon6 + lon7 + lon8 + lon9 + lon10) / 10
    altitude = (alt1 + alt2 + alt3 + alt4 + alt5 + alt6 + alt7 + alt8 + alt9 + alt10) / 10
    print (latitude)  # 23.0664710008
    print (longitude)  # 113.383369912
    print (altitude)  # -81.5852592469


latitude = 0.0000000
longitude = 0.0000000
altitude = 0.0000000
seq = 0
step = 500

def gps_sub_cb(gps):
    global latitude, longitude, altitude, seq, step
    latitude += gps.latitude
    longitude += gps.longitude
    altitude += gps.altitude
    seq += 1
    if step != 0:
        step -= 1
    else:
        lat = latitude/seq
        lon = longitude/seq
        alt = altitude/seq
        step = 500
        print ("\naverage: " + str(seq))
        print ('lat: ' + str(lat))
        print ('lon: ' + str(lon))
        print ('alt: ' + str(alt))



if __name__ == '__main__':
    rospy.init_node('fifty_round_up', anonymous=True)
    gps_sub = rospy.Subscriber("/uav3/dji_sdk/gps_position", NavSatFix, gps_sub_cb)
    start_point()
    goal_point()
    rospy.spin()

