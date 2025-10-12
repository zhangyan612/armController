#!/bin/bash
# CAN setup script

# CAN0 - 1 Mbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# CAN1 - 500 kbps
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 loopback off
sudo ip link set can1 up

# CAN2 - 500 kbps
sudo ip link set can2 down
sudo ip link set can2 type can bitrate 500000 loopback off
sudo ip link set can2 up

# CAN3 - 500 kbps
sudo ip link set can3 down
sudo ip link set can3 type can bitrate 500000 loopback off
sudo ip link set can3 up
