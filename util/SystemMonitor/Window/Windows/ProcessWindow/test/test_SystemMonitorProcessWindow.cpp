/*! \file test_SystemMonitorProcessWindow.cpp
 */
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    ProcessWindow window(logger);
    EXPECT_FALSE(window.setRecords({}));

    {
        WindowSize size = window.getWindowSize();
        EXPECT_GT(size.min_height_pixel, 0);
        EXPECT_GT(size.min_width_pixel, 0);
    }
    {
        std::vector<std::shared_ptr<IRecord>> records = window.getRecords();
        EXPECT_GT(records.size(), 0);
        for (auto record : records) { EXPECT_GT(record->getFields().size(), 0); }
    }
    {
        KeyMap noKey;
        EXPECT_TRUE(window.keyPressed(noKey));
    }
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
