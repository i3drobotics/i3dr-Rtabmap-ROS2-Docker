#!/bin/bash

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if X server is installed
if ! command -v X &> /dev/null; then
    echo "X server is not installed. Please install an X server."
    exit 1
fi

# Start X server
xhost +

# Run Docker container with GUI support
# May need to change --device paths depending on your system
docker run -it --name rtab \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/bus/002/004 \
    --device=/dev/bus/002/005 \
    humble-rtabmap-pyphase

# Revoke access to X server
xhost -
