# i3dr-Rtabmap-ROS2-Docker
Creating a Docker image to set up a ROS2 workspace that integrates pyphase and rtabmap.



## Ubuntu Host Machine

### Requirements
- Nvidia graphics card
- Nvidia drivers
- Virtualisation enabled in BIOS

### Install Docker
Install docker using these instructions: https://docs.docker.com/engine/install/ubuntu/

### Install Xorg
```
sudo apt-get install xorg openbox
```

### Checking Installs
Check that the correct nvidia drivers are installed with the following commands:
```
nvcc --version
nvidia-smi
cat /proc/driver/nvidia/version
```
Check Docker is installed correctly by running:
```
sudo docker run hello-world
```

### Building the Image
1. Ensure docker is running.
2. Run [build.sh](build.sh)

### Running a Container from the Image
1. Run [run.sh](run.sh).
2. Inside the container ensure you are in /root/ros2_ws and run:
```
source /opt/ros/foxy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
source /root/data/lic_setup.sh
```

```
cd /root/ros2_ws
source /opt/ros/foxy/setup.bash
git clone --branch foxy https://github.com/ros-perception/image_pipeline.git src/image_pipeline
git clone --branch foxy https://github.com/ros-perception/image_common.git src/image_common
git clone --branch foxy-devel https://github.com/ros-perception/perception_pcl.git src/perception_pcl
git clone https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
apt-get update
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
source /root/data/lic_setup.sh
```

### Test the camera
Inside the container navigate to ~/data and run:
```
python3 pyphase_example.py
```

### Running a RTabMap Scan
Inside the container navigate to ~/ros2_ws and run:
```
ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py left_serial:=40098272 right_serial:=40098282 camera_name:=I3DRTitania_746974616e24317 device_type:=titania interface_type:=usb exposure:=10000
```
CTRL-C to stop the scan.
CTRL-D to quit the docker image.



## Windows Host Machine
Note: Running an ubuntu docker container with a Windows host machine will not allow external devices (i.e. cameras) to connect. 

### Downloads
1. Docker desktop
2. Xming

#### Installing Docker
Check that your system supports virtualization by opening the command prompt and running:
```
systeminfo
```
If your processor supports Hardware Virtualization technology, you will be able to see a section of Hyper-V requirements along with the status. If virtualization is turned off, you will see “No” in front of the option “Virtualization Enabled in Firmware”. To allow virtualization you may need to enable it in BIOS.

With that done, install Docker desktop. To check it has installed correctly open the command prompt and run:
```
docker version
```

### Building the Image
1. Ensure docker is running (in windows there will be an icon in the system tray).
2. Run [build.bat](build.bat)

This only needs to be done once as the image will be stored locally.

### Running a Container from the Image
1. Run Xming. When it is running you will see an X icon in the system tray.
2. Start the container with [run.bat](run.bat).
3. Inside the container ensure you are in /root/ros2_ws and run:
```
source /opt/ros/foxy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
source /root/data/lic_setup.sh
```

## Developer Notes
Initially this was built with the line `FROM introlab3it/rtabmap_ros:humble`. This simplified the Dockerfile and allowed us to use Ubuntu 22.04 and ROS2 Humble, but the image did not utilize the GPU, which is necessary for running I3DRSGM. The current implementation uses the GPU and OpenGL, however it is only possible to use Ubuntu 20.04 and ROS2 Foxy.

To allow the I3DRSGM license to be accepted, it is necessary to run the container with the `--hostname yourlicensehostname` flag, so that the hostnbame of the container matches the hostname of the license. It also requires you to match the hostid of the license (only the first six characters seem to be necessary) with the hostid of the docker container. To do this the hostid of the container can be indirectly set by changing the ip address in the hosts file. `echo "yourlicenseipaddress yourlicensehostname" > /etc/hosts`. "yourlicenseipaddress" can be found by converting the hostid of the license to an ip address (see [license_scripts](license_scripts)).