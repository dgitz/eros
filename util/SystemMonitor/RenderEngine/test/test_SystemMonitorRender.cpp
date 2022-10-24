/*! \file test_SystemMonitorEngine.cpp
 */
#include <eros/Logger.h>
#include <eros/SystemMonitor/RenderEngine/RenderEngine.h>
#include <eros/SystemMonitor/RenderEngine/RenderWindow.h>
#include <eros/SystemMonitor/Window/Windows/DeviceWindow/DeviceWindow.h>
#include <eros/SystemMonitor/Window/Windows/InfoWindow/InfoWindow.h>
#include <eros/SystemMonitor/Window/Windows/NodeDiagnosticsWindow/NodeDiagnosticsWindow.h>
#include <eros/SystemMonitor/Window/Windows/ProcessWindow/ProcessWindow.h>
#include <eros/SystemMonitor/Window/Windows/StatusWindow/StatusWindow.h>
#include <gtest/gtest.h>
#include <stdio.h>

#include <memory>
using namespace eros;
std::map<eros::IWindow::WindowType, IWindow*> initializeWindows(Logger* logger) {
    std::map<eros::IWindow::WindowType, IWindow*> windows;
    {
        StatusWindow* window = new StatusWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::STATUS, window));
    }

    {
        ProcessWindow* window = new ProcessWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::PROCESS, window));
    }
    {
        InfoWindow* window = new InfoWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::INFO, window));
    }
    {
        DeviceWindow* window = new DeviceWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::DEVICE, window));
    }
    {
        NodeDiagnosticsWindow* window = new NodeDiagnosticsWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::NODEDIAGNOSTICS, window));
    }

    return windows;
}
TEST(BasicTest, RenderWindowTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    WindowSize size;
    RenderWindow window(logger, size, 0, 0);
    EXPECT_TRUE(window.init());
    delete logger;
}

TEST(BasicTest, RenderEngineTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    std::map<eros::IWindow::WindowType, IWindow*> windows = initializeWindows(logger);

    RenderEngine engine(logger, windows);
    EXPECT_TRUE(engine.initScreen());
    {  // Test Focus Increment
        for (int i = 0; i < 5 * (uint8_t)IWindow::WindowType::END_OF_LIST; ++i) {
            EXPECT_TRUE(engine.incrementFocus());
            uint16_t focusedWindowCount = 0;
            for (auto win : engine.getWindows()) {
                if (win.second.windowRender->isFocused() == true) {
                    focusedWindowCount++;
                }
            }
            EXPECT_EQ(focusedWindowCount, 1);
        }
    }
    { EXPECT_TRUE(engine.update(0.1, windows)); }
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
