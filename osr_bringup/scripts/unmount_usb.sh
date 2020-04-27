#!/bin/sh

if grep -qs '/media/data ' /proc/mounts; then
    umount /media/data
fi