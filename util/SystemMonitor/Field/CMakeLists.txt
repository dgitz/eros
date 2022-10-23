add_library(baseField BaseField.cpp)
target_link_libraries(baseField ${catkin_LIBRARIES})

add_library(genericField GenericField.cpp)
target_link_libraries(genericField ${catkin_LIBRARIES})

add_subdirectory(test)