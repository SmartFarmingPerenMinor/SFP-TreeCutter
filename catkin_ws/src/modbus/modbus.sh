#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash

cd ../../ && catkin build
source ./devel/setup.bash
rosrun modbus node
