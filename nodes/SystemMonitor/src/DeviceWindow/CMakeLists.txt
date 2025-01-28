add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} deviceWindow)