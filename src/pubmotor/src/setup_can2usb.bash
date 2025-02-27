#!/bin/bash

# enable kernel module: gs_usb
#sudo modprobe gs_usb

# bring up can interface
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 up

# install can utils
# sudo apt install -y can-utils
# sudo ip link set up can0