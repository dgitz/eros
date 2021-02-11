catkin_add_gtest(test_definitions test_definitions.cpp)
target_link_libraries(test_definitions ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_diagnostics test_diagnostics.cpp)
target_link_libraries(test_diagnostics ${catkin_LIBRARIES} ${Boost_LIBRARIES})