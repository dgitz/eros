cmake_minimum_required(VERSION 2.8.3)
project(eros)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)


include_directories(include)

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
  controlgroup.msg
  leverarm.msg
  encoder.msg
  timesyncinfo.msg
  subsystem_diagnostic.msg

 )
## Generate services in the 'srv' folder
add_service_files(
   FILES
    srv_connection.srv
    srv_device.srv
    srv_leverarm.srv
 )
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


