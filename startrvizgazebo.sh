#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash


./startgazebo.sh &
sleep 15


roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true &
sleep 8

roslaunch ur10e_moveit_config moveit_rviz.launch &
sleep 15
