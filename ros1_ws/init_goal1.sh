#!/bin/bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.1.111:11311


rostopic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: now, frame_id: "map"}, pose: { pose: {position: {x: 6.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}, } }'

rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 6.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}'