#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash


./startgazebo.sh &
sleep 15

rvizConfig=$(pwd)"/configs/rviz_ur10e_config.rviz"
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true rviz_config="$rvizConfig" &
sleep 15
