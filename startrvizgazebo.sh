#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash


./startgazebo.sh &
sleep 15

<<<<<<< HEAD

roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true &
sleep 8

roslaunch ur10e_moveit_config moveit_rviz.launch &
=======
rvizConfig=$(pwd)"/configs/rviz_ur10e_config.rviz"
roslaunch ur10e_moveit_config moveit_rviz.launch config:=true rviz_config="$rvizConfig" &
>>>>>>> 8f1d052760c4ef8ad572d90f5beec98544983296
sleep 15
