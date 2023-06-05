FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]
WORKDIR /ros2_ws

COPY . .


RUN apt update && apt upgrade -y && \
    # install build tools
    apt install -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions \
    # building nav2
    source /opt/$ROS_DISTRO/setup.bash && \
    git clone --branch $ROS_DISTRO https://github.com/ros-planning/navigation2.git src/ && \
    rm -rf  /etc/ros/rosdep/sources.list.d/20-default.list && \
    git -C src/ sparse-checkout set \
        navigation2/** \
        nav2_mppi_controller/** \
        nav2_bringup/** \
        nav2_msgs/** && \
    rosdep init && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --rosdistro $ROS_DISTRO && \
    sed -i 's/target_link_libraries(\${lib} xtensor xtensor::optimize xtensor::use_xsimd)/target_link_libraries(\${lib} xtensor xtensor::use_xsimd)/' src/nav2_mppi_controller/CMakeLists.txt && \
    # colcon build --symlink-install --executor sequential && \
    colcon build --symlink-install && \
    # clean to make the image smaller
    export SUDO_FORCE_REMOVE=yes && \
	apt remove -y \
        git \
        build-essential \
        python3-rosdep \
        python3-colcon-common-extensions && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(cat /ros2_ws/src/navigation2/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt
