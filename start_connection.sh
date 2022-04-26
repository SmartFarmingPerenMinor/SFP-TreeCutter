#!/bin/env bash

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

path_calibration=`pwd`"/configs/my_robot_calibration.yaml"

if [ -z "$1" ]
then
    roslaunch ur_calibration calibration_correction.launch robot_ip:=10.42.0.2 target_filename:="$path_calibration" &
    sleep 10 && roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=10.42.0.2 kinematics_config:="$path_calibration"
elif [[ $1 =~ ^(([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5]))\.){3}([1-9]?[0-9]|1[0-9][0-9]|2([0-4][0-9]|5[0-5])) ]]
    then
    roslaunch ur_calibration calibration_correction.launch robot_ip:=$1 target_filename:="$path_calibration" &
    sleep 10 && roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=$1 kinematics_config:="$path_calibration"
else 
    echo "invalid ip addrass!"
fi
