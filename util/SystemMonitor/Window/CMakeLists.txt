add_library(baseWindow BaseWindow.cpp)
target_link_libraries(baseWindow ${catkin_LIBRARIES})

add_library(windowTable WindowTable.cpp)
target_link_libraries(windowTable ${catkin_LIBRARIES} baseWindow)

add_library(windowText WindowText.cpp)
target_link_libraries(windowText ${catkin_LIBRARIES} baseWindow)

add_subdirectory(Windows)
add_subdirectory(test)