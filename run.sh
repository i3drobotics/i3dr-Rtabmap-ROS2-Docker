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

# Prompt user whether to run the docker container with --rm
read -p "Do you want to run the Docker container with --rm? (y/n): " run_rm
if [[ $run_rm == "y" ]]; then
    rm_flag="--rm"
else
    rm_flag=""
fi

# Prompt user whether to include the two --device
read -p "Do you want to include the two --device flags? (y/n): " include_device
if [[ $include_device == "y" ]]; then
    device_flags="--device ${camera_paths[0]} --device ${camera_paths[1]}"
else
    device_flags=""
fi

# Start X server
xhost +

# those following 3 lines would need to be done only one time
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Run Docker container with GUI support
# May need to change --device paths depending on your system
sudo docker run -it \
    $rm_flag \
    --name rtab \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e XAUTHORITY=$XAUTH \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    $device_flags \
    --hostname I3DRWL004 \
    humble-rtabmap-pyphase \
    /bin/bash -c "export ROS_NAMESPACE=rtabmap && rosrun rtabmap_viz rtabmap_viz"

# sudo docker run -it \
#     $rm_flag \
#     --name rtab \
#     --gpus all \
#     -e DISPLAY=$DISPLAY \
#     -e QT_X11_NO_MITSHM=1 \
#     -e NVIDIA_VISIBLE_DEVICES=all \
#     -e NVIDIA_DRIVER_CAPABILITIES=all \
#     -e XAUTHORITY=$XAUTH \
#     --runtime=nvidia \
#     --network host \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     $device_flags \
#     --hostname I3DRWL004 \
#     humble-rtabmap-pyphase

# Revoke access to X server
xhost -
