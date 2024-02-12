#!/bin/bash

# Get the directory path where the script is located
script_dir=$(dirname "$(realpath "$0")")

# Construct the full path to the calibration data folder
calibration_data_folder="$script_dir/calibrations"

# Check if the folder exists
if [ ! -d "$calibration_data_folder" ]; then
    echo "Error: Calibration data folder not found!"
    exit 1
fi

# Copy the calibration data folder into the Docker container
docker cp "$calibration_data_folder" rtab:/root/data/calibrations

# Check if the copy operation was successful
if [ $? -eq 0 ]; then
    echo "Calibration data copied successfully into rtab Docker environment."
    echo "Source folder: $calibration_data_folder"
    echo "Destination folder: rtab:/root/data/calibrations"
else
    echo "Error: Failed to copy calibration data into rtab Docker environment."
    exit 1
fi
