add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} deviceWindow)
add_dependencies(deviceWindow eros_generate_messages_cpp)