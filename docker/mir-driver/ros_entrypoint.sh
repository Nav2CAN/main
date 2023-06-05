#!/bin/bash
set -e

# setup ros environment
source /opt/ros/noetic/setup.bash --
source /ws/devel/setup.bash --

exec "$@"