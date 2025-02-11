/*! \file test_armdisarm_monitor.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "ArmDisarmMonitor.h"
using namespace eros;
using namespace eros_nodes;
TEST(BasicTest, TestOperation) {
    ArmDisarmMonitor SUT("Test", ArmDisarmMonitor::Type::SIMPLE);
    EXPECT_TRUE(false);
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
