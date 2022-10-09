catkin_add_gtest(test_systemmonitor_process test_systemmonitor_process.cpp)
target_link_libraries(test_systemmonitor_process ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger utility systemMonitorProcess)
add_dependencies(test_systemmonitor_process eros_generate_messages_cpp)

#add_rostest_gtest(test_systemMonitor
#    test_systemMonitor.test
#    test_systemMonitor.cpp
#)
#add_dependencies(test_systemMonitor system_monitor)
#target_link_libraries(test_systemMonitor ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)