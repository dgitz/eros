#!/bin/bash
cd /home/robot/catkin_ws
source devel/setup.bash
catkin_make -j2 >/dev/null
