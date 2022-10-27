/*! \file test_SystemMonitorDeviceManager.cpp
 */
#include <eros/Logger.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/Device.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceManager.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
double COMMTIMEOUT_S = 10.0;
TEST(BasicTest, TestDevice) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    Device SUT(logger, "SampleDevice", COMMTIMEOUT_S);
    double currentTime = 0.0;
    EXPECT_EQ(SUT.getStatus(), Level::Type::UNKNOWN);
    {
        eros::loadfactor lf;
        lf.DeviceName = SUT.getName();
        lf.stamp.fromSec(currentTime);
        EXPECT_FALSE(SUT.new_loadfactor(lf));
        lf.loadfactor = {0.0, 0.0, 0.0};
        EXPECT_TRUE(SUT.new_loadfactor(lf));
        EXPECT_TRUE(SUT.getStatus() <= Level::Type::NOTICE);

        eros::loadfactor bad_lf;
        bad_lf.DeviceName = "A totally different device name";
        EXPECT_FALSE(SUT.new_loadfactor(bad_lf));
        EXPECT_TRUE(SUT.getStatus() <= Level::Type::NOTICE);

        EXPECT_TRUE(SUT.update(currentTime += COMMTIMEOUT_S + COMMTIMEOUT_S / 10.0));
        EXPECT_TRUE(SUT.getLastHeartbeatDelta() > COMMTIMEOUT_S / 2.0);
        EXPECT_TRUE(SUT.getStatus() > Level::Type::NOTICE);

        lf.stamp.fromSec(currentTime += COMMTIMEOUT_S + COMMTIMEOUT_S / 8.0);
        EXPECT_TRUE(SUT.new_loadfactor(lf));
        EXPECT_TRUE(SUT.getStatus() <= Level::Type::NOTICE);
    }
    {
        eros::resource res;
        res.Name = SUT.getName();
        res.stamp.fromSec(currentTime += 2.0 * COMMTIMEOUT_S);
        EXPECT_TRUE(SUT.new_resourceavailable(res));

        eros::resource bad_res;
        res.Name = "A totally different device name";
        EXPECT_FALSE(SUT.new_resourceavailable(bad_res));
    }
    EXPECT_TRUE(SUT.getStatus() <= Level::Type::NOTICE);
    logger->log_debug(SUT.pretty());
    EXPECT_TRUE(SUT.update(currentTime += 1.1 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getStatus(), Level::Type::WARN);
    EXPECT_TRUE(SUT.update(currentTime += 5.0 * COMMTIMEOUT_S));
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.getStatus(), Level::Type::ERROR);

    EXPECT_TRUE(SUT.update(currentTime += 3.0 * COMMTIMEOUT_S));
    EXPECT_TRUE(SUT.getLastHeartbeatDelta() > COMMTIMEOUT_S / 2.0);
    delete logger;
}
TEST(BasicTest, TestDeviceManager) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    DeviceManager SUT(logger, COMMTIMEOUT_S);
    EXPECT_EQ(SUT.getDevices().size(), 0);
    {  // loadfactor
        // Device 1
        eros::loadfactor lf1;
        lf1.DeviceName = "Device1";
        lf1.loadfactor = {0.0, 0.0, 0.0};
        EXPECT_TRUE(SUT.new_loadfactor(lf1));
        EXPECT_EQ(SUT.getDevices().size(), 1);
        EXPECT_TRUE(SUT.new_loadfactor(lf1));
        EXPECT_EQ(SUT.getDevices().size(), 1);
        // Device 1
        eros::loadfactor lf2;
        lf2.DeviceName = "Device2";
        lf2.loadfactor = {0.0, 0.0, 0.0};
        EXPECT_TRUE(SUT.new_loadfactor(lf2));
        EXPECT_EQ(SUT.getDevices().size(), 2);
        EXPECT_TRUE(SUT.new_loadfactor(lf2));
        EXPECT_EQ(SUT.getDevices().size(), 2);
    }
    {  // resourceAvailable
        // Device 1
        eros::resource res1;
        res1.Name = "Device1";
        EXPECT_TRUE(SUT.new_resourceavailable(res1));
        EXPECT_EQ(SUT.getDevices().size(), 2);
        EXPECT_TRUE(SUT.new_resourceavailable(res1));
        EXPECT_EQ(SUT.getDevices().size(), 2);

        // Device 3
        eros::resource res3;
        res3.Name = "Device3";
        EXPECT_TRUE(SUT.new_resourceavailable(res3));
        EXPECT_EQ(SUT.getDevices().size(), 3);
        EXPECT_TRUE(SUT.new_resourceavailable(res3));
        EXPECT_EQ(SUT.getDevices().size(), 3);
    }
    logger->log_debug(SUT.pretty());
    delete logger;
}
TEST(DeviceManager, SupportFunctions) {
    {  // Sanitize DeviceName
        std::string normalDeviceName = "/abcd";
        EXPECT_EQ("abcd", DeviceManager::sanitizeDeviceName(normalDeviceName));

        std::string emptyDeviceName = "";
        EXPECT_EQ("", DeviceManager::sanitizeDeviceName(emptyDeviceName));

        std::string weirdDeviceName = "/a/b/c";
        EXPECT_EQ("a/b/c", DeviceManager::sanitizeDeviceName(weirdDeviceName));
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
