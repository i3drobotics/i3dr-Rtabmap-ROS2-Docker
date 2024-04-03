#!/bin/bash

# Start X server
xhost +

# Run Docker container with GUI support
sudo docker run -it \
    --rm \
    --name frtab \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    --hostname I3DRWL004 \
    foxy-rtabmap-pyphase

# Revoke access to X server
xhost -
