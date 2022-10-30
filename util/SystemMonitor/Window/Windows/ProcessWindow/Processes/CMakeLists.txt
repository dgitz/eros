add_library(genericProcess GenericProcess.cpp)
target_link_libraries(genericProcess ${catkin_LIBRARIES} baseProcess)
add_dependencies(genericProcess ${eros_EXPORTED_TARGETS})

add_library(erosProcess EROSProcess.cpp)
target_link_libraries(erosProcess ${catkin_LIBRARIES} baseProcess)
add_dependencies(erosProcess ${eros_EXPORTED_TARGETS})
add_subdirectory(test)