add_library(processWindow ProcessWindow.cpp)
target_link_libraries(processWindow ${catkin_LIBRARIES} logger windowTable)
add_subdirectory(test)