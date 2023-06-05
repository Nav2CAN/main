#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
ros_realsense_setup="/ros_ws/install/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"
echo "sourcing   $ros_realsense_setup"
source "$ros_realsense_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"
