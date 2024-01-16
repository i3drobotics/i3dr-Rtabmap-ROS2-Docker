#!/bin/bash

# Use lsusb to list USB devices and filter for your camera information
camera_info=$(lsusb | grep "Basler")

# Extract the bus and device numbers from the lsusb output for each camera
camera_paths=()
while read -r line; do
    bus=$(echo "$line" | awk '{print $2}')
    device=$(echo "$line" | awk '{print $4}' | tr -d ':')
    path="/dev/bus/usb/$bus/$device"
    camera_paths+=("$path")
done <<< "$camera_info"

# Print the paths
for ((i=0; i<${#camera_paths[@]}; i++)); do
    echo "Camera $((i+1)) path: ${camera_paths[i]}"
done

# Start X server
xhost +

# Run Docker container with GUI support
# May need to change --device paths depending on your system
sudo docker run -it --name rtab \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device "${camera_paths[0]}" \
    --device "${camera_paths[1]}" \
    humble-rtabmap-pyphase

# Revoke access to X server
xhost -
