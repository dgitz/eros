cmake_minimum_required(VERSION 2.8.3)
project(eros)
## Compile as C++11, supported in ROS Kinetic and newer
SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib_msgs
  rosbag
  cmake_modules
)
find_package(TinyXML REQUIRED)
find_package(Curses REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)

