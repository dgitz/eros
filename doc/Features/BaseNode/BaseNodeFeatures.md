# Base Node Process Features:
* Diagnostic Handler: Enable Diagnostics for different Diagnostic Types.  Software can update a specific diagnostic without losing context to other diagnostics.  For example, if a node is executing (DiagnosticType=SOFTWARE), it can just be at an INFO level.  But if the node is having trouble communicating with a sensor (DiagnosticType=SENSORS) that diagnostic can be at a WARN level.  Both of these diagnostics (and more) can be set at the same time.

# Base Node Features:
* Timing Loops: Has multiple timing loops that can be used to execute user-defined content at regular intervals.  Includes Loops at rates:
  * 10 Hz
  * 1 Hz
  * .1 Hz
  * Noisy .1 Hz (When system is time sync'd, all all timing loops will run at the same time and rate.  This can lead to problems when multiple processes are executing content at the same time causing brief high CPU usage.  This loop runs at .1 Hz with a random time offset to alleviate this problem.
  * .01 Hz
  * 3 User configured Loop Rates (Loop1, Loop2, Loop3)
* Instantiating Logger
* Continuously publishes a Node Heartbeat
* Instantiating a Node Process (user defined code that can be more readily unit tested without the ROS message framework involved).
* Uses Interrupt Handlers to kill node (no more hanging node executions when you hit Ctrl-C!)
* Run Node defined Unit Tests while Node is running!  Can be used for troubleshooting.
* Reports Resource Usage of a Node (CPU, RAM in perc)
* Request diagnostics active on a Node
* Request current Firmware version
* Change Logger Level
* Request Node State Change

# Future Features
