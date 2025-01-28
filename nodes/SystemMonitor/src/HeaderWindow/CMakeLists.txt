add_library(headerWindow HeaderWindow.cpp)
target_link_libraries(headerWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES} baseWindow)