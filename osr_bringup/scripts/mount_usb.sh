#!/bin/sh

if grep -qs '/media/data ' /proc/mounts; then
    echo "Drive already mounted"
else
    sudo mount -t ext4 /dev/sda1 /media/data
fi