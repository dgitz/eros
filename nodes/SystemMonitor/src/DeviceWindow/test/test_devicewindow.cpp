/*! \file test_devicewindow.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "DeviceWindow/DeviceWindow.h"
#include "WindowDefinitions.h"
using namespace eros_nodes::SystemMonitor;
TEST(BasicTest, Test_BasicOperation) {
    EXPECT_TRUE(false);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}