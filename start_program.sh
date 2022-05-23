#!/bin/env bash

source /opt/ros/noetic/setup.bash
source ./catkin_ws/devel/setup.bash

roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true &
sleep 8

roslaunch ur10e_moveit_config moveit_rviz.launch &
sleep 8

roslaunch realsense2_camera l515.launch &
sleep 8

rosrun tf2_ros static_transform_publisher 0 0 0 1.5708 -1.5708 0 wrist_3_link camera_link &
sleep 8

