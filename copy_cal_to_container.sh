#!/bin/bash

# Get the directory path where the script is located
script_dir="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Construct the full path to the calibration data folder
calibration_data_folder="$script_dir/calibration"

# Check if the folder exists
if [ ! -d "$calibration_data_folder" ]; then
    echo "Error: Calibration data folder not found!"
    exit 1
fi

echo "Source folder: $calibration_data_folder"
echo "Destination folder: rtab:/root/ros2_ws/install/phase_rtabmap_ros2/share/phase_rtabmap_ros2/cal"

# Copy the contents of the calibration data folder into the Docker container
docker cp "$calibration_data_folder/." rtab:/root/ros2_ws/install/phase_rtabmap_ros2/share/phase_rtabmap_ros2/cal

# Check if the copy operation was successful
if [ $? -eq 0 ]; then
    echo "Calibration data copied successfully into rtab Docker environment."
else
    echo "Error: Failed to copy calibration data into rtab Docker environment."
    exit 1
fi
