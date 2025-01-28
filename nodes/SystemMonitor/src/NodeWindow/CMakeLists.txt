add_library(nodeWindow NodeWindow.cpp)
target_link_libraries(nodeWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)