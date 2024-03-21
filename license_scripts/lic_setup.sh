#!/bin/bash

# Set the hostid by setting the corresponding ip address
echo "118.13.61.21 I3DRWL004" > /etc/hosts

# Copy all lic files from licenses to /usr/bin so that python can find it
cp ~/data/licenses/*.lic /usr/bin/