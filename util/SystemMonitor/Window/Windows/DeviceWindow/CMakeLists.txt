add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} windowTable)
add_subdirectory(test)