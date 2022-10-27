add_library(deviceManager Device.cpp DeviceManager.cpp)
target_link_libraries(deviceManager ${catkin_LIBRARIES} logger)
add_dependencies(deviceManager ${eros_EXPORTED_TARGETS})

add_library(deviceWindow DeviceWindow.cpp)
target_link_libraries(deviceWindow ${catkin_LIBRARIES} windowTable deviceManager)
add_subdirectory(test)