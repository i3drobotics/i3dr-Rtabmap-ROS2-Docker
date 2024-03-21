FROM introlab3it/rtabmap_ros:humble

SHELL ["/bin/bash", "-c"] 

RUN apt-get -y update

RUN apt-get install -y python3-pip

RUN sudo apt install wget

# Copy the license files from the host to the image
RUN mkdir -p /root/.i3dr/lic
COPY ./licenses /root/.i3dr/lic

# Get pyphase for Linux
RUN mkdir -p ~/pyphase310
RUN wget -P ~/pyphase310 https://github.com/i3drobotics/pyphase/releases/download/v0.3.0/phase-0.3.0-cp310-cp310-linux_x86_64.whl

# Install pyphase dependencies
RUN sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev
RUN sudo apt install -y libgl-dev liblapack-dev libblas-dev libgtk2.0-dev
RUN sudo apt install -y libgstreamer1.0-0 libgstreamer-plugins-base1.0-0
RUN sudo apt install -y zlib1g libstdc++6
RUN sudo apt install -y libc6 libgcc1

# Install pyphase and upgrade numpy
RUN python3 -m pip install ~/pyphase310/phase-0.3.0-cp310-cp310-linux_x86_64.whl
RUN python3 -m pip install numpy --upgrade

# Create ROS2 workspace
RUN mkdir -p ~/ros2_ws/src
RUN cd ~/ros2_ws
RUN source /opt/ros/humble/setup.bash

# Clone i3drobotics repo
RUN git clone --branch humble-devel https://github.com/i3drobotics/phase_rtabmap_ros2.git ~/ros2_ws/src/phase_rtabmap_ros2

# Install source folder into workspace
# colcon build requires older version of setuptools, otherwise setup.py doesn't work.
RUN python3 -m pip install --force-reinstall -v "setuptools==58.2.0"

# Install opencv
RUN python3 -m pip install opencv-python

# Add calibration files, licenses and pyphase_example to the image
RUN mkdir -p ~/data
RUN mkdir -p ~/data/pointclouds
WORKDIR /root/data
COPY ./pyphase_example.py pyphase_example.py
ADD ./calibration calibration
ADD ./licenses licenses

# Add and run a script that sets the hostid by setting the ip
COPY ./license_scripts/set_hostid_ip.sh set_hostid_ip.sh
RUN source ./set_hostid_ip.sh

# Change to ros2_ws for convenience
WORKDIR /root/ros2_ws

# colcon build gives "Duplicate package names not supported" error when building
# with Docker. However this error doesn't happen if you do the colcon build
# manually inside the container. Until this bug is fixed the following lines are done manually:
#RUN cd ~/ros2_ws
#RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#RUN source ~/ros2_ws/install/setup.bash