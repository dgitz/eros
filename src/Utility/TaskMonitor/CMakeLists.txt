#TaskMonitor Node
add_library(taskmonitor_node_process TaskMonitorNodeProcess.cpp)
target_link_libraries(taskmonitor_node_process ${catkin_LIBRARIES} ${CURSES_LIBRARIES} )
add_executable(taskmonitor_node taskmonitor_node.cpp)
target_link_libraries(taskmonitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${TinyXML_LIBRARIES} ${roscpp_LIBRARIES} ${CURSES_LIBRARIES} taskmonitor_node_process logger)
catkin_add_gtest(test_taskmonitor_node unit_tests/test_taskmonitor_node_process.cpp)
target_link_libraries(test_taskmonitor_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CURSES_LIBRARIES}  taskmonitor_node_process)
