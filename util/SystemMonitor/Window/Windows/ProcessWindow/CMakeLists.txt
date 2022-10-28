add_library(processManager Process.cpp ProcessManager.cpp)
target_link_libraries(processManager ${catkin_LIBRARIES} logger)
add_dependencies(processManager ${eros_EXPORTED_TARGETS})

add_library(processWindow ProcessWindow.cpp)
target_link_libraries(processWindow ${catkin_LIBRARIES} logger windowTable)
add_subdirectory(test)