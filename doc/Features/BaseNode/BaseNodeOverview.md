
# Call Order for Initialization
When using the BaseNodeProcess and the BaseNode, the proper call order is as follows:

NOTE: The following code will not compile, this is for illustrative purposes.

```
SampleNode *node = new SampleNode();  // Instantiate the Node
bool status = node->start(argc, argv);  // Call the Node Start Function (user provided, should follow a template)

start(...) {
  initialize_diagnostic(...) // Set the Root Diagnostic
  process = new SampleNodeProcess();  // Instantiate the Node Process
  initialize_firmware(...) // Set the Firmware Variables
  preinitialize_basenode(...) // Run pre-initialize Node content.  
  read_launchparameters()  // Node specific configuration 
  process->initialize(...)  // Initialize the Node Process
  process->enable_diagnostics(...) // Enable configured Diagnostic Types
  process->finish_initialization() // Finish initialization of the process
  finish_initialization() // Any other initialization that needs to happen at the end.
  request_statechange(Node::State::INITIALIZED);
  return;
}

preinitialize_basenode(...) {
Includes:
 * Initializing the ROS Node Handle
 * Node Heartbeat
 * Getting the hostname
 * Getting logger level from config
 * Getting loop rates from config
 * Setting up base node pubs, subs, service clients, etc.
}

read_launchparameters() {
Includes: 
 * Read node specific configuration values loaded from launch file.
 * Read node specific configuration values from other config files.
}
```
![](https://github.com/dgitz/eros/blob/master/doc/ResourceMonitor/output/ResourceMonitorClassDiagram.svg)
![](https://github.com/dgitz/eros/blob/master/doc/BaseNode/output/BaseNodeClassDiagram.svg)
![](https://github.com/dgitz/eros/blob/master/doc/CustomNode/output/CustomNodeClassDiagram.svg)
