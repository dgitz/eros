The EROS Logger has the following features:
* No ROS Dependencies (i.e. can be used in Nodes, Unit Tests, etc).
   * Can be used with ROS Console Output if Installed/Enabled.
* Logger initialized on a per node basis.
* Logger has a verbosity level (DEBUG,INFO,NOTICE,WARN,ERROR,FATAL) that can be changed at run-time.  When an item is requested to be logged at a certain verbosity level, and the logger has a configured verbosity level higher than the requested item, that item won't be logged.
* Logger writes to console window (can be disabled).  Console output is color coded: <= INFO: White, == NOTICE: Green, == WARN: Yellow, >= ERROR: Red
* Logger writes to text file output that gets reset when the logger is initialized.
* Logger output text file has the FILE and the LINE Number from the item that requested the log entry.
* All output is timestamped.
* Logger can take as an input either "std::string" or a Diagnostic.
* Can be used in a Node or in a Unit Test

![](https://github.com/dgitz/eros/blob/master/doc/Logger/output/LoggerClassDiagram.svg)
# Examples:
```
logger->log_notice("Hello World");
logger->log_debug("Another String of length: " + std::to_string(2));
logger->log_diagnostic(diag);
```

Console Print:
```
[16/02/2021 05:20:14 sample_node1]: NOTICE: Initialized.
[16/02/2021 05:20:14 sample_node1]: NOTICE: Running Node at Rate: 40.00 Hz.
[16/02/2021 05:20:14 sample_node1]: NOTICE: Configuration Files Loaded.
[16/02/2021 05:20:14 sample_node1]: INFO: Device:  System: ROVER Subsystem: ENTIRE_SYSTEM Component: ENTIRE_SUBSYSTEM Type: SOFTWARE Message: NOERROR Description: Node Configured.  Initializing.
[16/02/2021 05:20:14 sample_node1]: NOTICE: Node State: RUNNING
```

Text File Output:
```
[16/02/2021 05:20:14]: NOTICE: /home/robot/catkin_ws/src/eros/src/BaseNode.cpp(99) Initialized.
[16/02/2021 05:20:14]: NOTICE: /home/robot/catkin_ws/src/eros/src/BaseNode.cpp(190) Running Node at Rate: 40.00 Hz.
[16/02/2021 05:20:14]: NOTICE: /home/robot/catkin_ws/src/test_node/nodes/SampleNode/SampleNode.cpp(61) Configuration Files Loaded.
[16/02/2021 05:20:14]: INFO: /home/robot/catkin_ws/src/test_node/nodes/SampleNode/SampleNode.cpp(45) Device:  System: ROVER Subsystem: ENTIRE_SYSTEM Component: ENTIRE_SUBSYSTEM Type: SOFTWARE Message: NOERROR Description: Node Configured.  Initializing.
[16/02/2021 05:20:14]: NOTICE: /home/robot/catkin_ws/src/test_node/nodes/SampleNode/SampleNode.cpp(55) Node State: RUNNING
```