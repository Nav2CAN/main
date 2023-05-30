# The visualizer container for Little Helper
#
# Copyright (c) 2022 Karl Damkjær Hansen, Aalborg University

FROM ros:humble

# Install the remote desktop stuff
# - TigerVNC is a X server/frame buffer and VNC server in one. Earlier I used xvfb and
#   x11vnc in combination, but I had trouble letting the client resize the desktop.
#   TigerVNC solves this, plus it seems faster. 
# - xfce4 is a window manager. If we did not include this, we would be able to render
#   into X but there would be no windows, menus etc. I tried to include Gnome instead,
#   for a more familiar Ubuntu experience, but that took up a lot of space and wouldn't
#   actually run because it needed the systemd daemon, and that became too cumbersome.
# - novnc is a vnc client that connects to the VNC server and presents the desktop
#   in a web browser. This way the user does not need to have a pc with a VNC client
#   installed. We use the port 8080 for this. So that the user should type "<ip>:8080/vnc.html"
#   into their browser.
# - supervisor is a light weight process manager, a bit like systemd. We use it to launch
#   the above tools with the right settings. It also relaunches them if they crash.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        tigervnc-standalone-server \
        xfce4 \
        novnc \
        nginx-light \
        supervisor

# install ros package
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-rqt \
        ros-${ROS_DISTRO}-rqt-common-plugins

# install packages for visualizing our robot(s)
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ur-description && \
        rm -rf /var/lib/apt/lists/*
        # Remember this last line to clean up the image a little bit.

# Supervisord configuration
COPY littlehelper-viz.conf /etc/supervisor/conf.d/

# Web server configurations
COPY nginx.conf /etc/nginx/
COPY index.html /www/data/

# Wallpaper
COPY little_helper.png /usr/share/backgrounds/
COPY xfce4-desktop.xml /root/.config/xfce4/xfconf/xfce-perchannel-xml/

# Shortcuts
COPY rviz2.desktop /usr/share/applications
COPY rqt.desktop /usr/share/applications

ENV DISPLAY=:0

EXPOSE 80

CMD supervisord