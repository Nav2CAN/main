#!/bin/bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.1.111:11311

source /opt/ros/eloquent/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

rosparam load bridge.yaml

ros2 run ros1_bridge parameter_bridge
