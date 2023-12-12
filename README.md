# i3dr-Rtabmap-ROS2-Docker
Creating a Docker image to set up a ROS2 workspace that integrates pyphase and rtabmap.

## Downloads
1. Docker desktop
2. Xming

### Installing Docker
Check that your system supports virtualization by opening the command prompt and running:
```
systeminfo
```
If your processor supports Hardware Virtualization technology, you will be able to see a section of Hyper-V requirements along with the status. If virtualization is turned off, you will see “No” in front of the option “Virtualization Enabled in Firmware”. To allow virtualization you may need to enable it in BIOS.

With that done, install Docker desktop. To check it has installed correctly open the command prompt and run:
```
docker version
```

## Building the Image
1. Ensure docker is running (in windows there will be an icon in the system tray).
2. Run [build.bat](build.bat)

This only needs to be done once as the image will be stored locally.

## Running a Container from the Image
1. Run Xming. When it is running you will see an X icon in the system tray.
2. Start the container with [run.bat](run.bat), or [run_and_remove.bat](run_and_remove.bat) if you want the container to be automatically removed after you close Docker.
3. Inside the container change directory to the workspace with:
```
cd ~/dev_ws
```
4. Then build the ROS2 workspace with:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
5. Then run:
```
source ~/dev_ws/install/setup.bash
```