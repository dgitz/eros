add_library(baseRecord BaseRecord.cpp)
target_link_libraries(baseRecord ${catkin_LIBRARIES} baseField)

add_library(genericRecord GenericRecord.cpp)
target_link_libraries(genericRecord ${catkin_LIBRARIES} baseRecord)

add_subdirectory(test)