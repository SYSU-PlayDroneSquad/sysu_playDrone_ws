#!/bin/sh
catkin_make -DCATKIN_WHITELIST_PACKAGES="dji_sdk"
catkin_make -DCATKIN_WHITELIST_PACKAGES="hector_uav_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
echo "source ~/sysu_playDrone_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
