# Configuration
## Requirements
* System Master and Slave Launch files must have an argument: `device_hostname` defined (ROS_HOSTNAME is preferred but not required), like this:
```
<arg name="device_hostname" value="$(env ROS_HOSTNAME)"/>
```
* Node Launch files must have an argument: `robot_namespace` defined, like this:
```xml
<arg name="robot_namespace" value="MACH1" />
<include file="$(find some_package)/launch/some_launchfile.launch">
  <arg name="robot_namespace" value="$(arg robot_namespace)"/>
</include>
```
In the Node Launch config itself, the `robot_namespace` parameter must be set, like this:
```xml
<launch>
<arg name="robot_namespace"/> 
<node name="some_node_name" pkg="some_package" type="some_node" >
  <param name="robot_namespace" value="$(arg robot_namespace)"/>
</node>
</launch>
```