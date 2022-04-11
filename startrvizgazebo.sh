#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash


./startgazebo.sh &
sleep 15


roslaunch ur10_e_moveit_config moveit_rviz.launch config:=true &
sleep 15
