add_library(windows StatusWindow.cpp DeviceWindow.cpp InfoWindow.cpp NodeDiagnosticsWindow.cpp ProcessWindow.cpp)
target_link_libraries(windows ${catkin_LIBRARIES} windowText windowTable)