/*! \file test_SystemMonitorStatusWindow.cpp
 */
#include <eros/SystemMonitor/Window/Windows/StatusWindow/StatusWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
TEST(BasicTest, TestOperation) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    StatusWindow SUT(logger);
    EXPECT_FALSE(SUT.setRecords({}));

    {
        WindowSize size = SUT.getWindowSize();
        EXPECT_GT(size.min_height_pixel, 0);
        EXPECT_GT(size.min_width_pixel, 0);
    }
    {  // eros::armed_state
        eros::armed_state armedState;
        EXPECT_TRUE(SUT.newArmedState(armedState));
    }
    {  // current time
        double currentTime = 1.0;
        EXPECT_TRUE(SUT.set_currentROSTime(currentTime));
    }
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
TEST(SupportFunctionTest, SupportFunctions) {
    {  // convert Armed State to Color
        for (uint8_t i = 0; i < (uint8_t)ArmDisarm::Type::END_OF_LIST; ++i) {
            Color color = StatusWindow::convertArmedStateColor((ArmDisarm::Type)i);
            EXPECT_NE(color, Color::UNKNOWN);
            EXPECT_NE(color, Color::END_OF_LIST);
        }
    }
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
