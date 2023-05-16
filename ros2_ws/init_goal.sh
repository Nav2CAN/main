#!/bin/bash
source /opt/ros/eloquent/setup.bash
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{ header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: { pose: {position: {x: 30.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}, } }'



ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 36.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}'

