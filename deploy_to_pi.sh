#!/bin/bash

# Define the Raspberry Pi credentials and destination
PI_USER="pi"
PI_HOST="raspberrypi.local"
PI_DIR="/home/pi"

# Create the directory on the Raspberry Pi
ssh ${PI_USER}@${PI_HOST} "mkdir -p ${PI_DIR}"

# Copy the Raspberry Pi code to the Raspberry Pi
scp -r raspberry_pi/* ${PI_USER}@${PI_HOST}:${PI_DIR}

# Set up the 64MP Autofocus Synchronized Quad-Camera Kit
ssh ${PI_USER}@${PI_HOST} << 'EOF'
wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh

# Install libcamera dev and apps
./install_pivariety_pkgs.sh -p libcamera_dev
./install_pivariety_pkgs.sh -p libcamera_apps

# Install the Hawk-Eye kernel driver
./install_pivariety_pkgs.sh -p 64mp_pi_hawk_eye_kernel_driver

# Edit /boot/config.txt to add the necessary dtoverlay
if ! grep -q "dtoverlay=vc4-kms-v3d,cma-512" /boot/config.txt; then
    echo "dtoverlay=vc4-kms-v3d,cma-512" | sudo tee -a /boot/config.txt
fi

EOF

