add_library(renderWindow RenderWindow.cpp)
target_link_libraries(renderWindow ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

add_library(renderEngine RenderEngine.cpp)
target_link_libraries(renderEngine ${catkin_LIBRARIES} ${CURSES_LIBRARIES} renderWindow)

add_subdirectory(test)