Author: David Gitz
Task: Utility
Nodes:
A. taskmonitor_node
TODO:
- ping non-eROS nodes 
- system run time from timemaster (DONE)
- string formatting (width,color) (DONE)
- task state (DONE)
- kill node safely (DONE)
- node page through list (DONE)
- color formats are a bit weird
- add disk available to resource_available topic (DONE)
- display device resource available (DONE)
- generate snapshot/clear snapshot (DONE)
- show snapshot status (DONE)
- change task log level (DONE)
Usage: This node can be run on any device, on or off a Robot.
Purpose: 
1. Task Viewer

Unit Tests:
1.  Process Unit Tests:
  >>catkin_make run_tests_eros_gtest_test_taskmonitor_node_process

  
Development History:
31-May-2019 David Gitz

