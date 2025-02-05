eROS currently supports the following types of devices that can run ROS:
- Any x86_64 compatible system (uname -m gives: x86_64).  For this architecture, follow [x86 Development](../../README.md#setup-instructions).
- Any arm7 compatible system (uname -m gives: arv7l)  (Raspberry Pi...)
- Any aarch64 compatible system (uname -m gives: aarch64)  (Jetson Nano...)

Here are some basic assumptions:
1. All directories specified here and in the eROS content can be modified through either config files and in the code itself.  This just assumes the standard installation.

The following is a brief guide for setup operations on these different devices.

# ARM7VL
1. Add user to dialout group: `sudo usermod -a -G gpio <USER ACCOUNT>`
1. Set permissions on gpio by following this link: https://www.raspberrypi.org/forums/viewtopic.php?t=8999
1. Install the following dependencies:\
  `sudo apt install wiringpi python-smbus i2c-tools`
1. Follow the typical ROS Raspberry Pi Melodic (ros_comm) install instructions (use "~/catkin_ws" as your workspace): http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi
1. Install the following packages (by using the section "Adding Released Packages" as your guide): `rosbag actionlib sensor_msgs rosunit`
1. Depending on what type of device this is (i.e. lower end Raspberry Pi or higher end) you may or may not want to use this device to actually build content, just to use pre-built content.  Note that if you want to use as only a No Build Target, you will still need to have another device on your network that has the same architecture that can be used as a Build Server.
1. Create the necessary calls to perform startup scripts:  Edit the /etc/rc.local file and add the following line:\
`sudo /home/<USER ACCOUNT>/config/Startup/$(hostname)_Startup.sh`

## ARMV7L Build Server
1. Go to your workspace/src folder and clone eROS: \
  `git clone https://github.com/dgitz/eROS.git`
1. Perform either [Auto Setup](#auto-setup) or [Manual Setup](#manual-setup)
1. Go back to your catkin_ws directory
1. Install all dependencies, build eROS and run unit tests.\
  `rosdep install eros`\
  `catkin_make` \
  `catkin_make run_tests_eros`
## ARMV7L No Build Target
1. Go to your workspace/src folder and clone eROS: \
  `git clone --branch small_target https://github.com/dgitz/eROS.git`\
  `git clone --branch small_target https://github.com/dgitz/ros_hats.git`
1. Go back to your catkin_ws directory
1. Install all dependencies, build eROS and run unit tests.\
  `rosdep install eros`\
  `catkin_make`

# AARCH 
1. Follow the same setup steps as the X86_64 section.  You may or may not wish to add extra packages, typically the ROS bare-bones setup will be just fine.

# Auto Setup
1. Go to your workspace/src/eros/ folder and run:
`python scripts/simple_setup.py`

# Manual Setup
1. Create the following directories: \
  `mkdir -p ~/config/ # For General Config`\
  `mkdir -p ~/var/log/output/ # For Log output files`\
  `mkdir -p ~/storage/stage/ # Stage directory for device and system snapshots.`\
  `mkdir -p ~/storage/DATALOGS/ # Directory to store ROS bag files.`\
  `mkdir -p ~/storage/SNAPSHOT/DEVICESNAPSHOT/ # Directory to store generated Device Snapshots. `\
  `mkdir -p ~/storage/SNAPSHOT/SYSTEMSNAPSHOT/ # Directory to store generated System Snapshots. `\
1. Install the json parser:\
  `cd ~/`\
  `mkdir -p other_packages && cd other_packages/`\
  `git clone https://github.com/nlohmann/json.git`\
  `cd json`\
  `mkdir build && cd build`\
  `cmake ..`\
  `make`\
  `sudo make install`
1. Install the following dependencies: `sudo apt install libncurses5-dev libncursesw5-dev libtinyxml-dev gpsd libgps-dev gpsd-clients python-gps zip ntp`
