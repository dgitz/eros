# Objective
* Run various manual and automated tests to ensure consistency and robustness of content

# Test Cases
## Documentation Tests
| Test | Description | Checks |
| --- | --- | --- |
| Release Notes | Check Validity of Release Notes | <ul><li>Check that all stories and bugs completed are in Release Notes.</li></ul> |


## Regression Tests
| Test | Description | Checks |
| --- | --- | --- |
| Multiple Unit Tests | Run Unit Tests 10 times to look for intermittent failures. | <ul><li>All Unit Tests Pass</li></ul> |

### Instructions:
Run Regression Tests:<br>
`roscd eros`<br>
`./scripts/dev_tools.sh regression`

## Usage Tests
Applies To:
* Master Node
* Safety Node
* Diagnostics Node
* Datalogger Node
* Snapshot Node

| Test | Description | Checks |
| --- | --- | --- |
| Run individual Nodes | Run individual Node Launch Files. | <ul><li>All Nodes can be run individually.</li><li>All Nodes run without error.</li></ul>
| Run System Launch | All Nodes can be ran together via a single System Launch File. | <ul><li>System Launch starts all Nodes.</li><li>All Nodes run without error.</li></ul> |

### Instructions: Individual Node Tests
#### Terminal 1:
`cd <catkin workspace>`<br>
`roscore`
#### Terminal 2:
`cd <catkin workspace>`<br>
`roslaunch eros master_node.launch` <br>
>>> Look for any errors, then kill.

#### Terminal 3:
`cd <catkin workspace>`<br>
`roslaunch eros diagnostic_node.launch` <br>
>>> Look for any errors, then kill.


#### Terminal 4:
`cd <catkin workspace>`<br>
`roslaunch eros safety_node.launch` <br>
>>> Look for any errors, then kill.


#### Terminal 5:
`cd <catkin workspace>`<br>
`roslaunch eros datalogger_node.launch` <br>
>>> Look for any errors, then kill.


#### Terminal 5:
`cd <catkin workspace>`<br>
`roslaunch eros snapshot_node_master.launch` <br>
>>> Look for any errors, then kill.

#### Terminal 6:
`cd <catkin workspace>`<br>
`roslaunch eros snapshot_node_slave.launch` <br>
>>> Look for any errors, then kill.

### Instructions: System Launch Tests
`cd <catkin workspace>`<br>
`roslaunch eros launch_master.launch`


## Utility Tests
Applies To:
* System Monitor

| Test | Description | Checks |
| --- | --- | --- |
| Run System Monitor | Run System Launch Files and then run System Monitor. | <ul><li>System Monitor runs without error.</li></ul> |

### Instructions: System Monitor Tests
#### Terminal 1:
`cd <catkin workspace>`<br>
`roslaunch eros launch_master.launch`

#### Terminal 2:
`cd <catkin workspace>`<br>
`rosrun eros system_monitor`

## Performance Tests
Applies To:
* Master Node
* Safety Node
* Diagnostics Node
* Datalogger Node
* Snapshot Node
* System Monitor

### Instructions: Performance Tests
1. Change `datalogger_node.launch` parameter `SnapshotMode` to `false`
#### Terminal 1:
`cd <catkin workspace>`<br>
`roslaunch eros launch_master.launch`

>>> Let run for at least 4 hours, Then:
Follow: [Log Analysis](https://github.com/fastrobotics/eros/wiki/LogAnalysis) and Save Images to Wiki


| Test | Description | Checks |
| --- | --- | --- |
| Check for Memory leaks, CPU Usage | Run System Launch Files and then run System Monitor. | <ul><li>Run for 4 hours.</li><li>Validate CPU, RAM Usage from Data Logs.</li></ul>