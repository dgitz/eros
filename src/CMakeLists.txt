add_library(logger Logger.cpp)
target_link_libraries(logger ${catkin_LIBRARIES})
add_dependencies(logger eros_generate_messages_cpp)

add_library(baseNodeProcess BaseNodeProcess.cpp)
target_link_libraries(baseNodeProcess ${catkin_LIBRARIES} logger)
add_dependencies(baseNodeProcess eros_generate_messages_cpp)

add_subdirectory(test)
