/*! \file test_SystemMonitorEngine.cpp
 */
#include <eros/SystemMonitor/RenderEngine/RenderEngine.h>
#include <eros/SystemMonitor/RenderEngine/RenderWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;

TEST(BasicTest, RenderWindowTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    WindowSize size;
    RenderWindow window(logger, size, 0, 0);
    delete logger;
}

TEST(BasicTest, RenderEngineTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    RenderEngine engine(logger, {});
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
