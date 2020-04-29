#!/bin/sh

if grep -qs '/media/data ' /proc/mounts; then
    sudo umount /media/data
fi