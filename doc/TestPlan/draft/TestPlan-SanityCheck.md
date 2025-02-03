# Test Plan: Sanity Check
## Test Plan Variables
| Variable | Description |
| --- | --- |
| EROS_BRANCH | The branch to checkout in eros

## Assumptions
1. ROS environment is setup:
   - ROS Noetic is installed
   - ROS network environment variables are defined

## Setup
1. Checkout Source Code
```code
cd ~/git/
rm -rf test_workspaces/
mkdir -p test_workspaces/catkin_ws/src/ && cd test_workspaces/catkin_ws/src/
git clone https://github.com/dgitz/eros
git checkout <EROS_BRANCH>
git branch
```
>>> Should yield <EROS_BRANCH>

2. Build
```code
cd ~/git/test_workspaces/catkin_ws/
source /opt/ros/noetic/setup.bash
catkin_make
```
>>> All software should build with no issues.

3. Run Unit Tests and Verify Results
```code 
catkin_make -j1 run_tests
catkin_test_results build/test_results/
```
>>> Should yield no failing tests.

4. Verify ROS package built and setup correctly
```code
roscd eros
```
>>> Should change directory automatically to the directory `~/git/test_workspaces/catkin_ws/src/eros`

## Tests
1. Start Launch Master
```code
cd ~/git/test_workspaces/catkin_ws/
source devel/setup.bash
roslaunch eros launch_master.launch
```
>>> All console output for Nodes is at a NOTICE or Lower.

>>> No nodes crashed

2. Start System Monitor
Open a new terminal and run:
```code
cd ~/git/test_workspaces/catkin_ws/
source devel/setup.bash
rosrun eros system_monitor
```

>>> The following Nodes are listed:
- Master Node
- Diagnostics Node
- Datalogger Node
- Safety Node
- Snapshot Node

>>> All nodes are in a RUNNING state

