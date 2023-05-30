FROM ros:noetic

# install ros package
RUN apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-mir-robot \
    && rm -rf /var/lib/apt/lists/*
