#!/bin/bash

( cd ./catkin_ws && catkin_make )

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

roslaunch ur_gazebo ur10e_bringup.launch limited:=false &
sleep 8

