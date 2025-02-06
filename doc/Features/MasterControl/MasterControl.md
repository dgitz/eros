[Features](../Features.md)

# Master Control

## Overview
Master Control is a feature that provides a level of coordination, that is useful especially in systems that include multiple ROS devices.

## Master Node
An eROS Master Node is avaialable that provides the following features:

### Features
The Master Node performs the following:
* Reads device info.
* Creates a service to pass this device info to other nodes.
* Reports device resource usage availability (individual nodes report their resource usage, the master reports what's left).
* Reports device load factor values (1,5 and 15 min)

### Configuration
- 1 Master Node should be ran on each device in an EROS compatible system. 