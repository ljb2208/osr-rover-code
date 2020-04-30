#!/bin/bash

source /home/lbarnett/catkin_ws/devel/setup.sh

# source this when you launch everything on a local machine
export ROS_MASTER_URI=http://localhost:11311
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"