/*! \file test_SystemMonitorProcessWindow.cpp
 */
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessWindow SUT(logger);
    EXPECT_FALSE(SUT.setRecords({}));

    {
        WindowSize size = SUT.getWindowSize();
        EXPECT_GT(size.min_height_pixel, 0);
        EXPECT_GT(size.min_width_pixel, 0);
    }
    // Give it some data
    double currentTime_s = 0.0;
    double dt = ProcessWindow::COMMTIMEOUT_S / 1000.0;
    {
        {  // Process 1: Generic Process

            std::string nodeName = "Process1";
            EXPECT_TRUE(SUT.new_nodeAlive("Host1", nodeName, currentTime_s += dt));
        }
        {  // Process 2: EROS Process

            std::string nodeName = "Process2";
            eros::heartbeat msg;
            msg.NodeName = nodeName;
            msg.stamp.fromSec(currentTime_s += dt);
            EXPECT_TRUE(SUT.new_heartbeat(msg));

            eros::resource res;
            res.Name = nodeName;
            msg.stamp.fromSec(currentTime_s += dt);
            EXPECT_TRUE(SUT.new_resource(res));
        }
    }

    // Tests
    if (1) {
        std::vector<std::shared_ptr<IRecord>> records = SUT.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }
    EXPECT_TRUE(SUT.update(100.0 * ProcessWindow::COMMTIMEOUT_S));
    if (1) {
        std::vector<std::shared_ptr<IRecord>> records = SUT.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }

    KeyMap noKey;
    EXPECT_TRUE(SUT.keyPressed(noKey));

    delete logger;
}
TEST(ProcessWindowSupportFunctions, SupportFunctionTests) {
    // Smallify ROS Name
    {
        std::string name = "/abc";
        EXPECT_EQ("/abc", ProcessWindow::smallifyROSName(name));
    }
    {
        std::string name = "/a/b";
        EXPECT_EQ("/.../b", ProcessWindow::smallifyROSName(name));
    }
    {
        std::string name = "/a/b/c/defg";
        EXPECT_EQ("/.../defg", ProcessWindow::smallifyROSName(name));
    }
    {
        std::string name = "/123245/67890/112233445555666/77777777777777";
        EXPECT_EQ("/.../77777777777777", ProcessWindow::smallifyROSName(name));
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
