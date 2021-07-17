catkin_add_gtest(test_definitions test_definitions.cpp)
target_link_libraries(test_definitions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(test_definitions eros_generate_messages_cpp)

catkin_add_gtest(test_logger test_logger.cpp)
target_link_libraries(test_logger ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger)
add_dependencies(test_logger eros_generate_messages_cpp)

catkin_add_gtest(test_diagnostics test_diagnostics.cpp)
target_link_libraries(test_diagnostics ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger)
add_dependencies(test_diagnostics eros_generate_messages_cpp)

catkin_add_gtest(test_resourcemonitor test_resourcemonitor.cpp)
target_link_libraries(test_resourcemonitor ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger resourceMonitor)
add_dependencies(test_resourcemonitor eros_generate_messages_cpp)

catkin_add_gtest(test_base test_base.cpp)
target_link_libraries(test_base ${catkin_LIBRARIES}  ${Boost_LIBRARIES} baseNodeProcess)
add_dependencies(test_base eros_generate_messages_cpp)

add_rostest_gtest(test_baseNode
    test_baseNode.test
    test_baseNode.cpp
)
add_dependencies(test_baseNode baseNode)
target_link_libraries(test_baseNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger baseNode)