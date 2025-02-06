[Features](../Features.md)

# Snapshot

## Overview
Snapshots are useful for logging various information in a system to look for issues, aid in development, etc.  eROS provides a Snapshot Node that performs this functionality.

## Snapshot Node

### Features
- Supports "Device Snapshots" and "System Snapshots".  As a robot may include multiple devices, each device can generate a snapshot, and the System Snapshot is the combined snapshot of the entire system.
- System Commands: A configuration file is available that includes running arbitrary commands on each device and logging the relevant output.  For example, you can look at CPU performance (`top -bn1`), Get the latest syslog report, get a list of all processes running (`ps aux`), etc.
- Files/Folders: A configurable list of files and/or folders is available that will dump whatever contents are in these into the device snapshot.
- Architecture Support: The configuration to define what commands, files, and folders to include in the device snapshots is defined at the Architecture level, as typically different platforms can put files in diffrent locations on the file system, have different command line support, etc.
- The System Snapshot includes ROS bags that are snapshotted themselves, which allows only storing bags when events occurred.  NOTE that this requires running the Data Logger Node.

### Configuration
In the Snapshot Config xml file, add all the devices that should report a device snapshot:
```xml
<SnapshotConfigFile>
    <SnapshotConfig>
        <SnapshotDevices>
            <Device>DevComputer1</Device>
            <Device>DevModule1</Device>
            <Device>BuildServer1</Device>
            <Device>ControlModule2</Device>
            ...
        </SnapshotDevices>
        ....
    </SnapshotConfig>
</SnapshotConfigFile>
```
