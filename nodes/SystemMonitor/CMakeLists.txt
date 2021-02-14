add_library(systemMonitorProcess SystemMonitorProcess.cpp)
target_link_libraries(systemMonitorProcess ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseNodeProcess)

add_executable(system_monitor systemmonitor_node.cpp) 
target_link_libraries(system_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURSES_LIBRARIES} systemMonitorProcess baseNode )

add_subdirectory(test)