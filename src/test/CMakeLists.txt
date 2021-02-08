catkin_add_gtest(test_definitions test_definitions.cpp)
target_link_libraries(test_definitions ${catkin_LIBRARIES} ${Boost_LIBRARIES})
