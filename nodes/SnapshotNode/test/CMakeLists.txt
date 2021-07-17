catkin_add_gtest(test_snapshotProcess test_SnapshotProcess.cpp)
target_link_libraries(test_snapshotProcess ${catkin_LIBRARIES} ${Boost_LIBRARIES} logger snapshotProcess)
add_dependencies(test_snapshotProcess eros_generate_messages_cpp)

add_rostest_gtest(test_snapshotNode
    test_snapshotNode.test
    test_snapshotNode.cpp
)
add_dependencies(test_snapshotNode snapshot_node)
target_link_libraries(test_snapshotNode ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${GTEST_LIBRARIES} logger)