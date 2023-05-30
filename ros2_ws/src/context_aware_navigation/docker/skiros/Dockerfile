FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
        tigervnc-standalone-server \
        openbox \
        supervisor \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:0

WORKDIR /skiros_ws/src

RUN apt-get update \
    && apt-get install -y \
        git \
        python3-pip  \
        python3-catkin-tools \
        wget \
    && source /opt/ros/noetic/setup.bash \
    && catkin_init_workspace \
    && git clone https://github.com/RVMI/skiros2 \
    && git clone https://github.com/RVMI/skiros2_std_lib \
    && git clone https://github.com/RVMI/skiros2_examples \
    && apt-get update \
    && rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y \
    && apt-get install -y \
        ros-noetic-interactive-markers \
    && rm -rf /var/lib/apt/lists/* \
    && pip install -r skiros2/requirements.txt \
    && ./skiros2/skiros2/scripts/install_fd_task_planner.sh

WORKDIR /skiros_ws

RUN source /opt/ros/noetic/setup.bash \
    && catkin build \
    && sed -i '/source/c\source "/skiros_ws/devel/setup.bash" --' /ros_entrypoint.sh

# Supervisord configuration
COPY skiros-viz.conf /etc/supervisor/conf.d/

CMD ["supervisord"]
#CMD ["roslaunch", "skiros2", "skiros.launch"]