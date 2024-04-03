FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

ENV DEBIAN_FRONTEND=noninteractive

# Setup nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# CUDA variables required JIT compilation
# CUDA_CACHE_PATH should be used to attach a volume for caching the JIT compilation
ENV CUDA_CACHE_MAXSIZE=2147483647
ENV CUDA_CACHE_DISABLE=0
ENV CUDA_CACHE_PATH=/root/.nv/ComputeCache

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

# Install pylon
RUN wget https://www2.baslerweb.com/media/downloads/software/pylon_software/pylon_7_4_0_14900_linux_x86_64_debs.tar.gz && \
    tar -xvf pylon_7_4_0_14900_linux_x86_64_debs.tar.gz && \
    dpkg -i pylon_7.4.0.14900-deb0_amd64.deb && \
    rm -rf pylon_7_4_0_14900_linux_x86_64_debs.tar.gz pylon_7.4.0.14900-deb0_amd64.deb

# RUN wget https://bugs.launchpad.net/~ubuntu-security-proposed/+archive/ubuntu/ppa/+build/18845128/+files/libicu55_55.1-7ubuntu0.5_amd64.deb && \
#     wget http://security.ubuntu.com/ubuntu/pool/universe/x/xerces-c/libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
#     wget https://launchpad.net/~ubuntu-security/+archive/ubuntu/ppa/+build/15108504/+files/libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
#     dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb && \
#     dpkg -i libxerces-c3.1_3.1.3+debian-1_amd64.deb && \
#     dpkg -i libpng12-0_1.2.54-1ubuntu1.1_amd64.deb && \
#     rm -rf libicu55_55.1-7ubuntu0.5_amd64.deb libxerces-c3.1_3.1.3+debian-1_amd64.deb libpng12-0_1.2.54-1ubuntu1.1_amd64.deb

# Install ROS 2 Foxy dependencies
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

# Create dev workspace
RUN mkdir -p /root/dev_ws/src
WORKDIR /root/dev_ws

# Install phase
RUN mkdir -p /root/phase
WORKDIR /root/phase
RUN wget https://github.com/i3drobotics/phase/releases/download/v0.3.0/phase-v0.3.0-ubuntu-20.04-x86_64.deb
RUN sudo apt update && apt install -f -y --no-install-recommends ./phase-v0.3.0-ubuntu-20.04-x86_64.deb
#RUN sudo apt install ./phase-v0.3.0-ubuntu-20.04-x86_64.deb
WORKDIR /root/dev_ws

# Get pyphase for Linux
RUN mkdir -p /root/pyphase38
RUN wget -P /root/pyphase38 https://github.com/i3drobotics/pyphase/releases/download/v0.3.0/phase-0.3.0-cp38-cp38-linux_x86_64.whl

# Install pyphase dependencies
RUN apt-get update
RUN apt install -y libavcodec-dev libavformat-dev libswscale-dev
RUN apt install -y libgl-dev liblapack-dev libblas-dev libgtk2.0-dev
RUN apt install -y libgstreamer1.0-0 libgstreamer-plugins-base1.0-0
RUN apt install -y zlib1g libstdc++6
RUN apt install -y libc6 libgcc1

# Install pip
RUN apt-get install -y python3-pip

# Install pyphase and then delete wheel file 
RUN python3 -m pip install /root/pyphase38/phase-0.3.0-cp38-cp38-linux_x86_64.whl \
    && rm -r /root/pyphase38

# upgrade numpy, install opencv
RUN python3 -m pip install numpy --upgrade
RUN python3 -m pip install opencv-python

# Install missing pyphase dependency
RUN wget https://github.com/i3drobotics/phobosIntegration/releases/download/v1.0.54/libicu55_55.1-7ubuntu0.5_amd64.deb
RUN dpkg -i libicu55_55.1-7ubuntu0.5_amd64.deb
RUN rm libicu55_55.1-7ubuntu0.5_amd64.deb

# wget https://github.com/basler/pypylon/releases/download/3.0.1/pypylon-3.0.1-cp38-cp38-manylinux_2_31_x86_64.whl
# python3 -m pip install pypylon-3.0.1-cp38-cp38-manylinux_2_31_x86_64.whl

# wget https://github.com/basler/pypylon/releases/download/2.3.0/pypylon-2.3.0-cp38-cp38-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
# python3 -m pip install pypylon-2.3.0-cp38-cp38-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
# pip install sphinx sphinx-rtd-theme setuptools wheel twine flake8 pytest pybind11-stubgen numpydoc

# Clone i3drobotics repo
RUN git clone --branch foxy-devel https://github.com/i3drobotics/phase_rtabmap_ros2.git /root/dev_ws/src/phase_rtabmap_ros2

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
COPY ./pyphase_example.py pyphase_example.py
COPY ./plotter.py plotter.py
ADD ./calibration calibration
ADD ./licenses licenses

# Add a script that sets the hostid by setting the ip
COPY ./license_scripts/lic_setup.sh lic_setup.sh

WORKDIR /root/dev_ws

# Manually run:
#source /opt/ros/foxy/setup.bash
#rosdep update && rosdep install --from-paths src --ignore-src -r -y
#export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
#colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#. install/setup.bash
#source /root/data/lic_setup.sh