#!/bin/bash
bash -c ". /home/lbarnett/catkin_ws/devel/setup.sh"
bash -c ". /home/lbarnett/catkin_ws/devel/setup.bash"
bash -c ". /opt/ros/melodic/setup.sh"
bash -c ". /opt/ros/melodic/setup.bash"
bash -i -c "roslaunch osr_bringup xavier.launch"
