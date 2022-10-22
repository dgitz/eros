add_library(renderEngine RenderEngine.cpp)
target_link_libraries(renderEngine ${catkin_LIBRARIES} ${CURSES_LIBRARIES})

add_subdirectory(test)