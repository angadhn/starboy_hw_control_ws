#!/bin/bash
export ROS_IP=$(hostname -I)
export ROS_HOSTNAME=$ROS_IP
export ROS_MASTER_IP=192.168.0.11:11311
export ROS_MASTER_URI=http://192.168.0.11:11311

source devel/setup.bash
roslaunch starboy_software.launch pipeline:=ompl
