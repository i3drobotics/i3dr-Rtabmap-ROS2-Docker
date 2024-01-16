#!/bin/bash

# Start X server
xhost +

# Run Docker container with GUI support
# May need to change --device paths depending on your system
sudo docker run -it --name rtab \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/bus/002/003 \
    --device=/dev/bus/002/004 \
    humble-rtabmap-pyphase

# Revoke access to X server
xhost -
