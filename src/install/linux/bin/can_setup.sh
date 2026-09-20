#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

if [ -e /dev/ttyACM0 ]; then
  chmod 666 /dev/ttyACM0
else
  echo "Warning: /dev/ttyACM0 not found"
fi

for i in 0 1 2 3; do
  ip link set down can$i
  ip link set can$i type can bitrate 1000000 loopback off
  ip link set up can$i
done