#!/bin/bash
rm /home/robot/catkin_ws/build/test_results/icarus_rover_v2/*.xml >/dev/null 2>/dev/null
cd /home/robot/catkin_ws
source devel/setup.bash
catkin_make run_tests_icarus_rover_v2 >/dev/null

