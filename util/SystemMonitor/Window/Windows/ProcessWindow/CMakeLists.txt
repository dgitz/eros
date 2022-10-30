add_library(baseProcess BaseProcess.cpp)
target_link_libraries(baseProcess ${catkin_LIBRARIES} logger)
add_dependencies(baseProcess ${eros_EXPORTED_TARGETS})

add_subdirectory(Processes)

add_library(processManager ProcessManager.cpp)
target_link_libraries(processManager ${catkin_LIBRARIES} logger baseProcess erosProcess genericProcess)
add_dependencies(processManager ${eros_EXPORTED_TARGETS})

add_library(processWindow ProcessWindow.cpp)
target_link_libraries(processWindow ${catkin_LIBRARIES} logger windowTable processManager)

add_subdirectory(test)