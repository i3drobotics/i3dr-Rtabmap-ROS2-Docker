#!/bin/bash

# Get the directory where the script is located
script_dir=$(dirname "$(realpath "$0")")

# URLs of the files to be downloaded
url_deb="https://github.com/i3drobotics/phase/releases/download/v0.3.0/phase-v0.3.0-ubuntu-20.04-x86_64.deb"
url_whl="https://github.com/i3drobotics/pyphase/releases/download/v0.3.0/phase-0.3.0-cp38-cp38-linux_x86_64.whl"

# File names extracted from URLs
file_deb="$script_dir/$(basename "$url_deb")"
file_whl="$script_dir/$(basename "$url_whl")"

# Function to download a file from a given URL
download_file() {
    local url="$1"
    local file="$2"
    echo "Downloading $file from $url..."
    curl -L -o "$file" "$url"
}

# Parse the arguments
force=false
if [[ "$1" == "--force" ]]; then
    force=true
fi

# Check and download files
if [ "$force" = true ] || [ ! -f "$file_deb" ]; then
    download_file "$url_deb" "$file_deb"
else
    echo "$file_deb already exists. Use --force to download again."
fi

if [ "$force" = true ] || [ ! -f "$file_whl" ]; then
    download_file "$url_whl" "$file_whl"
else
    echo "$file_whl already exists. Use --force to download again."
fi
