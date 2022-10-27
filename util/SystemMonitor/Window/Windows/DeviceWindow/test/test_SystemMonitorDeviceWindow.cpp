/*! \file test_SystemMonitorDeviceWindow.cpp
 */
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestBasicOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    DeviceWindow SUT(logger);
    EXPECT_FALSE(SUT.setRecords({}));

    {
        WindowSize size = SUT.getWindowSize();
        EXPECT_GT(size.min_height_pixel, 0);
        EXPECT_GT(size.min_width_pixel, 0);
    }
    // Give it some data
    {
        double currentTime = 1.0;
        eros::loadfactor lf_device1;
        lf_device1.DeviceName = "Device1";
        lf_device1.loadfactor = {0.0, 0.0, 0.0};
        lf_device1.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_loadfactor(lf_device1));
        EXPECT_TRUE(SUT.update(2.0));
    }
    logger->log_debug(SUT.pretty());
    {
        std::vector<std::shared_ptr<IRecord>> records = SUT.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }
    {
        KeyMap noKey;
        EXPECT_TRUE(SUT.keyPressed(noKey));
    }
    delete logger;
}
TEST(MoreFunctionalityTest, TestBasicOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    DeviceWindow SUT(logger);
    EXPECT_FALSE(SUT.setRecords({}));
    double currentTime = 0.0;
    double dt = DeviceWindow::DEVICE_COMMTIME_THRESHOLD / 20.0;
    // Give it some data
    {
        currentTime += dt;
        eros::loadfactor lf_device1;
        lf_device1.DeviceName = "Device1";
        lf_device1.loadfactor = {0.0, 0.0, 0.0};
        lf_device1.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_loadfactor(lf_device1));
        EXPECT_TRUE(SUT.update(currentTime += dt));
        EXPECT_EQ(SUT.getRecords().size(), 1);

        eros::loadfactor lf_device2;
        lf_device2.DeviceName = "Device2";
        lf_device2.loadfactor = {0.0, 0.0, 0.0};
        lf_device2.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_loadfactor(lf_device2));
        EXPECT_TRUE(SUT.update(currentTime += dt));
        EXPECT_EQ(SUT.getRecords().size(), 2);

        eros::loadfactor lf_device3;
        lf_device3.DeviceName = "Device3";
        lf_device3.loadfactor = {0.0, 0.0, 0.0};
        lf_device3.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_loadfactor(lf_device3));
        EXPECT_TRUE(SUT.update(currentTime += dt));
        EXPECT_EQ(SUT.getRecords().size(), 3);

        eros::loadfactor lf_device4;
        lf_device4.DeviceName = "Device4";
        lf_device4.loadfactor = {0.0, 0.0, 0.0};
        lf_device4.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_loadfactor(lf_device4));
        EXPECT_TRUE(SUT.update(currentTime += dt));
        EXPECT_EQ(SUT.getRecords().size(), 4);

        eros::resource res_device1;
        res_device1.Name = "Device1";
        res_device1.stamp.fromSec(currentTime);
        EXPECT_TRUE(SUT.new_resourceavailable(res_device1));
        EXPECT_TRUE(SUT.update(currentTime += dt));
        EXPECT_EQ(SUT.getRecords().size(), 4);
    }

    logger->log_debug(SUT.pretty());
    {
        std::vector<std::shared_ptr<IRecord>> records = SUT.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }
    EXPECT_TRUE(SUT.update(currentTime += 5.0 * DeviceWindow::DEVICE_COMMTIME_THRESHOLD));
    logger->log_debug(SUT.pretty());
    {
        std::vector<std::shared_ptr<IRecord>> records = SUT.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
