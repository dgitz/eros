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
class TesterStatusWindow : public WindowText
{
   public:
    TesterStatusWindow(eros::Logger* logger) : WindowText(logger, IWindow::WindowType::STATUS) {
    }
    virtual ~TesterStatusWindow() {
    }
    WindowSize getWindowSize() {
        WindowSize size;
        ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 0.0, 0.0, 100.0, 15.0);
        size.coordinate = coord;
        size.min_height_pixel = 1;
        size.min_width_pixel = 1;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        std::vector<std::shared_ptr<IRecord>> records;
        {
            std::vector<std::shared_ptr<IField>> fields;
            std::shared_ptr<GenericRecord> record(new GenericRecord);
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = "Status";
            data.startCoordinate.start_x_pixel = 0;
            data.startCoordinate.start_y_pixel = 0;
            field->setData(data);
            fields.push_back(std::move(field));

            record->setFields(fields);
            records.push_back(std::move(record));
        }

        return records;
    }
    bool keyPressed(KeyMap key) {
        (void)key;
        logger->log_warn("NOT SUPPORTED YET.");
        return true;
    }
    // Not allowed to set records independently.
    bool setRecords(std::vector<std::shared_ptr<IRecord>> records) {
        (void)records;
        return false;
    }

   private:
};
class TesterInfoWindow : public WindowText
{
   public:
    TesterInfoWindow(eros::Logger* logger) : WindowText(logger, IWindow::WindowType::INFO) {
    }
    virtual ~TesterInfoWindow() {
    }
    WindowSize getWindowSize() {
        WindowSize size;
        ScreenCoordinatePerc coord(CoordinateReference::GLOBAL, 33.0, 75.0, 33.0, 25.0);
        size.coordinate = coord;
        size.min_height_pixel = 1;
        size.min_width_pixel = 1;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        std::vector<std::shared_ptr<IRecord>> records;
        {
            std::vector<std::shared_ptr<IField>> fields;
            std::shared_ptr<GenericRecord> record(new GenericRecord);
            std::shared_ptr<GenericField> field(new GenericField);
            RenderData data;
            data.data = "Info";
            data.startCoordinate.start_x_pixel = 0;
            data.startCoordinate.start_y_pixel = 0;
            field->setData(data);
            fields.push_back(std::move(field));

            record->setFields(fields);
            records.push_back(std::move(record));
        }

        return records;
    }
    bool keyPressed(KeyMap key) {
        (void)key;
        logger->log_warn("NOT SUPPORTED YET.");
        return true;
    }
    // Not allowed to set records independently.
    bool setRecords(std::vector<std::shared_ptr<IRecord>> records) {
        (void)records;
        return false;
    }

   private:
};
std::map<eros::IWindow::WindowType, IWindow*> initializeWindows(Logger* logger) {
    std::map<eros::IWindow::WindowType, IWindow*> windows;
    {
        TesterStatusWindow* window = new TesterStatusWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::STATUS, window));
    }

    {
        TesterInfoWindow* window = new TesterInfoWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::INFO, window));
    }
    {
        TesterInfoWindow* window = new TesterInfoWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::DEVICE, window));
    }
    {
        TesterInfoWindow* window = new TesterInfoWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::NODEDIAGNOSTICS, window));
    }
    {
        TesterInfoWindow* window = new TesterInfoWindow(logger);
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::PROCESS, window));
    }

    return windows;
}
TEST(BasicTest, RenderWindowTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    WindowSize size;
    RenderWindow window(logger, size, 100, 100);
    EXPECT_TRUE(window.init());
    delete logger;
}

TEST(BasicTest, RenderEngineTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    std::map<eros::IWindow::WindowType, IWindow*> windows = initializeWindows(logger);

    RenderEngine engine(logger, windows);

    ASSERT_TRUE(engine.initScreen());
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
