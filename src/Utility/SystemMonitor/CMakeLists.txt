#SystemMonitor Node
add_library(systemmonitor_node_process SystemMonitorNodeProcess.cpp)
target_link_libraries(systemmonitor_node_process ${catkin_LIBRARIES} ${CURSES_LIBRARIES} )
add_executable(systemmonitor_node systemmonitor_node.cpp)
target_link_libraries(systemmonitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} ${CURSES_LIBRARIES} systemmonitor_node_process logger)
catkin_add_gtest(test_systemmonitor_node unit_tests/test_systemmonitor_node_process.cpp)
target_link_libraries(test_systemmonitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURSES_LIBRARIES}  systemmonitor_node_process)
