A Snapshot Node is ran on every ROS Device.  

# Features
* Generates a Device Snapshot by sending a message to /SystemCommand with a Command of GENERATE_SNAPSHOT
* Runs the configured commands in the xml file and outputs to a file, when the Device Snapshot is generated collects this information into a zip file.
* Configuration file is Architecture specific to allow for different commands to be ran.
* Configuration file allows arbitrary file and folder copy to device snapshot.
* System Snapshot includes ROS Bag files from Data Logger Node
# Configuration
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

# Future Features