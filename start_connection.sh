#!/bin/env bash

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

path_calibration=`pwd`"/my_robot_calibration.yaml"
echo "$path_calibration"

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.137.2 target_filename:="$path_calibration" &
sleep 10 && roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.137.2 kinematics_config:="$path_calibration"