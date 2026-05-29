#!/bin/bash

set -euo pipefail

REMOTE_IP="${1:-192.168.1.157}"
BASE_PORT="${2:-14000}"

echo "========================================="
echo " Initializing 4 SocketCAN virtual buses  "
echo "========================================="

sudo modprobe vcan

for i in 0 1 2 3
do
    if ! ip link show "vcan${i}" > /dev/null 2>&1; then
        sudo ip link add dev "vcan${i}" type vcan
        echo "Created vcan${i}"
    fi

    sudo ip link set up "vcan${i}"
    echo "Brought up vcan${i}"
done

echo ""
echo "========================================="
echo " Creating Cannelloni UDP tunnels         "
echo " Remote IP: ${REMOTE_IP}"
echo "========================================="

for i in 0 1 2 3
do
    port=$((BASE_PORT + i))
    cannelloni -I "vcan${i}" -R "${REMOTE_IP}" -r "${port}" -l "${port}" > /dev/null 2>&1 &
    echo "Started tunnel for vcan${i} on UDP ${port}"
done

echo ""
echo "All CAN FD bridge processes are running in background."
echo "Use 'pkill cannelloni' to stop them."
