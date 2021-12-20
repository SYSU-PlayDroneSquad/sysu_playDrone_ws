#!/bin/sh
source /opt/ros/kinetic/setup.sh
source ~/sysu_playDrone_ws/devel/setup.bash
gnome-terminal --tab -e "roslaunch dji_sdk sdk.launch"
sleep 10s
gnome-terminal --tab -e "roslaunch ground_control_station command_parser.launch"
sleep 25s
gnome-terminal --tab -e "roslaunch cloud_server_communication client.launch"
