FROM introlab3it/rtabmap_ros:humble

#FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

# ENV DEBIAN_FRONTEND=noninteractive

# # Setup nvidia-docker hooks
# LABEL com.nvidia.volumes.needed="nvidia_driver"
# ENV PATH /usr/local/nvidia/bin:${PATH}
# ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# # CUDA variables required JIT compilation
# # CUDA_CACHE_PATH should be used to attach a volume for caching the JIT compilation
# ENV CUDA_CACHE_MAXSIZE=2147483647
# ENV CUDA_CACHE_DISABLE=0
# ENV CUDA_CACHE_PATH=/root/.nv/ComputeCache

# Install required software
RUN apt update && apt install -y --no-install-recommends \
        sudo \
        software-properties-common \
        ca-certificates \
        build-essential \
        cmake \
        git \
        curl \
        wget \
    && apt-get -y autoremove \
    && apt-get clean \
    # cleanup
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Install pip
RUN apt-get install -y python3-pip && \
    python3 -m pip install pip --upgrade

# Install pylon
RUN wget https://www2.baslerweb.com/media/downloads/software/pylon_software/pylon_7_4_0_14900_linux_x86_64_debs.tar.gz && \
    tar -xvf pylon_7_4_0_14900_linux_x86_64_debs.tar.gz && \
    dpkg -i pylon_7.4.0.14900-deb0_amd64.deb && \
    rm -rf pylon_7_4_0_14900_linux_x86_64_debs.tar.gz pylon_7.4.0.14900-deb0_amd64.deb

# Create dev workspace
ARG WORKSPACE_NAME=ros2_ws
RUN mkdir -p /root/${WORKSPACE_NAME}/src
WORKDIR /root/${WORKSPACE_NAME}

# Install phase
RUN wget https://github.com/i3drobotics/phase/releases/download/v0.3.0/phase-v0.3.0-ubuntu-20.04-x86_64.deb && \
    apt-get update --fix-missing && \
    apt-get install -f -y --no-install-recommends ./phase-v0.3.0-ubuntu-20.04-x86_64.deb && \
    rm -rf ./phase-v0.3.0-ubuntu-20.04-x86_64.deb

# Get pyphase for Linux
RUN wget -P /root/pyphase310 https://github.com/i3drobotics/pyphase/releases/download/v0.3.0/phase-0.3.0-cp310-cp310-linux_x86_64.whl

# Install pyphase dependencies
RUN apt-get update
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt-get install -y libgl-dev liblapack-dev libblas-dev libgtk2.0-dev
RUN apt-get install -y libgstreamer1.0-0 libgstreamer-plugins-base1.0-0
RUN apt-get install -y zlib1g libstdc++6
RUN apt-get install -y libc6 libgcc1

# Install pypylon
RUN python3 -m pip install pypylon

# Install pyphase and then delete wheel file 
RUN python3 -m pip install /root/pyphase310/phase-0.3.0-cp310-cp310-linux_x86_64.whl \
    && rm -r /root/pyphase310

# upgrade numpy, install opencv
RUN python3 -m pip install numpy --upgrade && \
    python3 -m pip install opencv-python

# Install missing pyphase dependency
RUN wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/libicu55_55.1-7ubuntu0.5_amd64.deb && \
    dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
    rm libicu55_55.1-7ubuntu0.5_amd64.deb

# Clone i3drobotics, ros, rtabmap repos
RUN apt-get update
RUN git clone --branch humble https://github.com/ros-perception/image_common.git /root/${WORKSPACE_NAME}/src/ros-perception/image_common
RUN git clone --branch humble https://github.com/ros-perception/image_pipeline.git /root/${WORKSPACE_NAME}/src/ros-perception/image_pipeline
RUN git clone https://github.com/introlab/rtabmap.git  src/rtabmap
RUN git clone --branch humble-devel https://github.com/introlab/rtabmap_ros.git /root/${WORKSPACE_NAME}/src/rtabmap_ros
RUN git clone --branch humble-devel https://github.com/i3drobotics/phase_rtabmap_ros2.git /root/${WORKSPACE_NAME}/src/phase_rtabmap_ros2

# Install rosdep and colcon
RUN apt-get update
RUN python3 -m pip install -U rosdep && rosdep init
RUN python3 -m pip install -U colcon-common-extensions
RUN python3 -m pip install pytest==7.2

# for matplotlib gui
RUN apt-get install -y python3-tk

# Add calibration files, licenses and pyphase_example to the image
RUN mkdir -p /root/data/pointclouds
WORKDIR /root/data
COPY ./example_scripts/pyphase_example.py pyphase_example.py
COPY ./example_scripts/plotter.py plotter.py
ADD ./calibration calibration
ADD ./licenses licenses

# Add pyphase utils wheel and install it
ADD ./pyphase_utils pyphase_utils
RUN python3 -m pip install ./pyphase_utils/pyphaseutils-1.0-py3-none-any.whl

# Add a script that sets the hostid by setting the ip
COPY ./license_scripts/lic_setup.sh lic_setup.sh

WORKDIR /root/${WORKSPACE_NAME}

# Manually run:
# source /opt/ros/humble/setup.bash
# sudo apt-get update
# rosdep update && rosdep install --from-paths src --ignore-src -r -y
# export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# . install/setup.bash
# source /root/data/lic_setup.sh