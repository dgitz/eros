# Heartbeat (Message)
A Heartbeat is published at approximately 10 Hz and includes the following information:
 - The timestamp when the heartbeat was generated.
 - The HostName of the device the node is running on.
 - The Base NodeName of the node.
 - The actual NodeName of the node.
 - The current Node::State (defined in eros_Definitions.h)

Example
```
$ rostopic echo /sample_node/heartbeat 
stamp: 
  secs: 1613389483
  nsecs: 250178248
HostName: "robot-Nitro-AN515-53"
BaseNodeName: "sample_node"
NodeName: "/sample_node"
NodeState: 4

```

# Get Node Diagnostics (Service)
Get the current Node Diagnostics.  A request is made with the following parameters:  Minimum Level and Diagnostic Type.  See eros_Definitions.h for more info.  

* Use 0 and 0 to to get all diagnostics reported by the node.
* Use the specific values to filter on (Minimum Level or Diagnostic Type).
* Use 255 to ignore (Minimum Level or Diagnostic Type).

For example, a Node that currently has 3 diagnosics with incrementing levels (INFO(2),NOTICE(3),WARN(4)) and 3 different Diagnostic Types (SOFTWARE(2),DATA_STORAGE(6),SYSTEM_RESOURCE(11)) would give the following based on the request:
* Request: MinLevel=0, DiagnosticType=0: All 3 Diagnostics
* Request: MinLevel=255, DiagnosticType=11: SYSTEM_RESOURCE Diagnostic only
* Request: MinLevel=3, DiagnosticType=255: All Diagnostics that have a Level of 3 and above.

# Logger Level (Service)
Change the current Logger Level.  A request is made with the following parameters: Verbosity (all CAPS).  Example:
```
$ rosservice call /sample_node/srv_loggerlevel WARN
Response: "Changed Logger Level to: WARN"
```
# Firmware Version (Service)
Get the Node Firmware. No extra parameters are needed.  Example:
```
$ rosservice call /sample_node/srv_firmware 
BaseNodeName: "sample_node"
NodeName: "/sample_node"
MajorRelease: 0
MinorRelease: 0
BuildNumber: 0
Description: "Latest Rev: 12-Feb-2021"

```