add_library(dataLoggerProcess DataLoggerProcess.cpp)
target_link_libraries(dataLoggerProcess ${catkin_LIBRARIES} baseNodeProcess)

add_executable(datalogger_node DataLoggerNode.cpp) 
target_link_libraries(datalogger_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} dataLoggerProcess baseNode )
add_subdirectory(test)
