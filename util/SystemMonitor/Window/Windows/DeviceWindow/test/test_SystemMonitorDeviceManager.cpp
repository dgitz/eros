/*! \file test_SystemMonitorDeviceManager.cpp
 */
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/Device.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceManager.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestDevice) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    Device SUT;
    delete logger;
}
TEST(BasicTest, TestDeviceManager) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    DeviceManager SUT;
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
