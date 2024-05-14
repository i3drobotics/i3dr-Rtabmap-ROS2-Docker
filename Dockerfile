FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04 as runtime

ARG DEBIAN_FRONTEND=noninteractive

# Uncomment the lines below to use a 3rd party repository
# to get the latest (unstable from mesa/main) mesa library version
# RUN apt-get update && apt install -y software-properties-common
# RUN add-apt-repository ppa:oibaf/graphics-drivers -y

# RUN apt update && apt install -y \
#     vainfo \
#     mesa-va-drivers \
#     mesa-utils

# ENV LIBVA_DRIVER_NAME=d3d12
# ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
# CMD vainfo --display drm --device /dev/dri/card0

RUN apt update && apt install -y
# Install curl, python, pip, and python3-tk for displaying matplotlib GUI
RUN apt install curl -y && \
    apt install wget -y && \
    apt install git -y && \
    apt install python3 -y && \
    apt install python3-pip -y && \
    apt install python3-tk -y

RUN pip3 install matplotlib

# Install ROS2 humble
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8
RUN apt install software-properties-common -y && \
    add-apt-repository universe -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade -y
RUN apt install ros-humble-desktop -y
SHELL ["/bin/bash", "-c"] 
RUN source /opt/ros/humble/setup.bash

# Install rosdep
RUN apt install python3-rosdep2 -y
RUN rosdep init; exit 0
# added exit 0 because rosdep init gives:
# "ERROR: default sources list file already exists: /etc/ros/rosdep/sources.list.d/20-default.list Please delete if you wish to re-initialize"
RUN rosdep update

# Get test matplotlib script
RUN mkdir /root/data
WORKDIR /root/data
COPY ./example_scripts/plotter.py plotter.py

# install pyphase
RUN apt install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt install -y libgl-dev liblapack-dev libblas-dev libgtk2.0-dev
RUN apt install -y libgstreamer1.0-0 libgstreamer-plugins-base1.0-0
RUN apt install -y zlib1g libstdc++6
RUN apt install -y libc6 libgcc1
RUN wget https://github.com/i3drobotics/pyphase/releases/download/v0.3.0/phase-0.3.0-cp310-cp310-linux_x86_64.whl && \
    pip3 install ./phase-0.3.0-cp310-cp310-linux_x86_64.whl && \
    rm ./phase-0.3.0-cp310-cp310-linux_x86_64.whl
RUN pip3 install pypylon

RUN apt install python3-colcon-common-extensions -y
RUN apt install cmake -y

SHELL ["/bin/sh", "-c"] 

# clone rtabmap and phase_rtabmap_ros2 and then build
RUN mkdir /root/ros2_ws
WORKDIR /root/ros2_ws
RUN git clone https://github.com/introlab/rtabmap.git src/rtabmap
RUN git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
RUN git clone --branch humble-devel https://github.com/i3drobotics/phase_rtabmap_ros2.git src/phase_rtabmap_ros2
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN export MAKEFLAGS="-j6"
#RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#RUN . install/setup.bash

# WIP