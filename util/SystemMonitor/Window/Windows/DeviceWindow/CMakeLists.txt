add_library(deviceManager Device.cpp DeviceManager.cpp)
target_link_libraries(deviceManager ${catkin_LIBRARIES})

add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} windowTable)
add_subdirectory(test)