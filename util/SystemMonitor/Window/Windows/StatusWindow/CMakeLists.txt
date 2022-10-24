add_library(statusWindow StatusWindow.cpp)
target_link_libraries(statusWindow ${catkin_LIBRARIES} logger windowText)
add_subdirectory(test)