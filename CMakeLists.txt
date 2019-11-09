cmake_minimum_required(VERSION 2.8.3)
project(eros)
set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions(-std=c++11 -Wall -Wextra -Wno-switch)
FIND_PACKAGE (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs 
  message_generation 
  std_msgs
  cmake_modules
  roscpp
  rospy
  std_msgs
  roslaunch
  )

find_package(Boost REQUIRED COMPONENTS system thread )
find_package(Curses)
add_message_files(
  DIRECTORY msg
  FILES
  test.msg
  signal.msg
  resource.msg
  diagnostic.msg
  device.msg
  pin.msg
  command.msg
  firmware.msg
  heartbeat.msg
  remotecontrol.msg
  imu.msg
  usermessage.msg
  battery.msg
  iopins.msg
  pose.msg
  leverarm.msg
  encoder.msg
  timesyncinfo.msg
  subsystem_diagnostic.msg
  systemsnapshot_state.msg
  loadfactor.msg
  uptime.msg
  tune_controlgroup.msg
  view_controlgroup.msg
  system_state.msg


 )
## Generate services in the 'srv' folder
add_service_files(
   FILES
    srv_connection.srv
    srv_device.srv
    srv_leverarm.srv
    srv_snapshotstate.srv
    srv_sql.srv
    srv_pin.srv
 )
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}  eros
  CATKIN_DEPENDS roscpp rospy std_msgs roslib
  DEPENDS Boost message_runtime
  
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
add_library(logger util/logger.cpp)
add_library(eros_math util/eros_math.cpp)
add_subdirectory(src/Utility)


