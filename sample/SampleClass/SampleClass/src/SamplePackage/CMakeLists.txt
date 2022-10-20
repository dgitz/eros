include_directories(
  ../../include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(sampleClass SampleClass.cpp)
target_link_libraries(sampleClass ${catkin_LIBRARIES} logger)
add_dependencies(sampleClass eros_generate_messages_cpp)
add_subdirectory(test)
