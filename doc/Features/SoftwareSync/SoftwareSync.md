[Features](../Features.md)

# Software Sync
Used to sync software to different devices.  Here are some features:
- Configurable directories to sync (currently hard-coded in script) that has the following types "Source","Config", and "Binary"
- Can specify to build on target device after syncing

# Configuration
1. Create a config directory (if not already done).  Default: `/home/robot/config/`
1. Copy the file "SyncConfig.xml" from the eROS/config/ directory to this new directory.
1. Add other Folder xml tags as needed to this file:
  * `Name` is a user defined Name, does not have to match anything.
  * `Type` is one of the following: "Config","Source","Binary".
    * "Config" is synced when syncing to a buildserver or a remote device.
    * "Source" is synced when syncing to a buildserver only.
    * "Binary" is synced when syncing to a remote device only.
  * `Directory` is the source (and target) directory for this content
  * `Architecture` is what device architecture is supported for this folder.  If the target device architecture does not match the architecture defined here, the folder won't get sync'd to the target device.  Note that you can add multiple Architectures to this list.
  
# Usage
## Sync Mode: build_server
- Essentially syncs all folders marked "Source" and "Config" to the specified target(s) and optionally builds them.  

## Sync Mode: buildserver_target
- The idea with this mode is to take advantage of one device's higher processing capabilities to act as a build server (that may or may not be the same architecture as the development system) and copy those binaries built on the build server to any other specified targets.  
- Sync's all folders marked "Source" and "Config" to first specified target, optionally builds them, and then the first target then syncs to all following targets with all folders marked "Config" and "Binary".  

Run the script:
```python3 scripts/syncSoftware.py --help``` 


## Future Features
- When syncing binaries, checks remote with origin architecture