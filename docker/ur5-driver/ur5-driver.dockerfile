FROM ros:humble

RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ur-robot-driver \
        && rm -rf /var/lib/apt/lists/*

CMD [ "ros2", "launch", "ur_robot_driver", "ur_control.launch.py", "ur_type:=ur5", "robot_ip:=192.168.12.148" ]
