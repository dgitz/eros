[Features](../Features.md)

# Safety Control

## Overview
Safety is an important part of a robotic system.  eROS enables better safety in a ROS environment by including a coordinated safety system.

## Arm & Disarm
Arming and Disarming are concepts used to enable movement of actuators in a robot, and to disable movement.  More detail on the eROS Arm&Disarm content is avaialable under [Arm & Disarm](ArmDisarm.md)

## Safety Node
An eROS Safety Node is avaiable that has the following features:

### Features
* Deciding the Armed State of the System

### Configuration
In the node launch file, add the following entries to have the Safety Node monitor topics to inform the arm/disarm system.
* Note that the number indicated in the _000 field should increment on each new topic, regardless of what type it is.
* At least 1 Topic is required for the system to go from DISARMED_CANNOTARM to DISARM.

#### Topics of type eros/ready_to_arm
 `<param name="ReadyToArm_Topic_000"   value="<TOPIC NAME>"/>`\
 `<param name="ReadyToArm_Type_000"   value="DEFAULT"/>`\
 `<param name="ReadyToArm_Topic_001"   value="<TOPIC NAME>/>`\
 `<param name="ReadyToArm_Topic_001"   value="DEFAULT"/>` 
  ...
#### Topics of type std_msgs/Bool
 `<param name="ReadyToArm_Topic_000"   value="<TOPIC NAME>"/>`\
 `<param name="ReadyToArm_Type_000"   value="SIMPLE"/>`\
 `<param name="ReadyToArm_Topic_001"   value="<TOPIC NAME>/>`\
 `<param name="ReadyToArm_Topic_001"   value="SIMPLE"/>` 
  ...