No more running ```rosbag record --all``` from the command line again!

The DataLogger Node can be used to record topics to a bag file.  This can work by either writing directly to the disk or by storing in RAM, and then a snapshot trigger is used to write to disk.

# Features
- No-Snapshot Mode: Will always write to disk.  
- Snapshot Mode: Only writes to disk when the topic ```/snapshot_trigger``` is triggered.  Note: This can eat up a large amount of RAM, especially with large ros data traffic.
- Snapshot file is created when trigger is generated, which includes information about the snapshot.
- Configurable directory where bag files are stored.

# Future Features
- Regex Include/Exclude topics
- Circular buffer for log files.
- Clear RAM buffer when usage is too high