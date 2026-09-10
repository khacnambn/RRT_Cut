FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    git \
    cmake \
    build-essential \
    libeigen3-dev \
    libompl-dev \
    ros-noetic-navigation \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-gmapping \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-rviz \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros_motion_planning_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /root/ros_motion_planning_ws/devel/setup.bash ]; then source /root/ros_motion_planning_ws/devel/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
