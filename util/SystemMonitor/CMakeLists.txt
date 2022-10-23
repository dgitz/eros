add_subdirectory(Window)
add_subdirectory(RenderEngine)
add_library(systemMonitorProcess SystemMonitorProcess.cpp)
target_link_libraries(systemMonitorProcess ${catkin_LIBRARIES}  baseNodeProcess)

add_executable(system_monitor SystemMonitor.cpp) 
target_link_libraries(system_monitor ${catkin_LIBRARIES} ${Boost_LIBRARIES} systemMonitorProcess baseNode windows renderEngine)
add_dependencies(system_monitor ${eros_EXPORTED_TARGETS})
add_subdirectory(test)
