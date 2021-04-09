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
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)
add_message_files(
  DIRECTORY msg
  FILES
  command.msg
  command_state.msg
  diagnostic.msg
  heartbeat.msg
  loadfactor.msg
  resource.msg
  subsystem_diagnostic.msg
  uptime.msg
  armed_state.msg
  mode_state.msg
  ready_to_arm.msg
)
add_service_files(
  FILES
  srv_device.srv
  srv_firmware.srv
  srv_logger_level.srv
  srv_get_diagnostics.srv
  srv_change_nodestate.srv
)
add_action_files(
  DIRECTORY action
  FILES
  system_command.action
  system_snapshot.action
)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(INCLUDE_DIRS include DEPENDS
      TinyXML actionlib_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)


add_subdirectory(src)
add_subdirectory(nodes)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Doxygen

# look for Doxygen package
find_package(Doxygen)

if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # Note: do not put "ALL" - this builds docs together with application EVERY TIME!
    add_custom_target( eros_docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
