add_library(infoWindow InfoWindow.cpp)
target_link_libraries(infoWindow ${catkin_LIBRARIES} logger windowText)
add_subdirectory(test)