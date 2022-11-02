/*! \file test_SystemMonitorWindow.cpp
 */
#include <eros/SystemMonitor/Field/GenericField.h>
#include <eros/SystemMonitor/Record/GenericRecord.h>
#include <eros/SystemMonitor/Window/BaseWindow.h>
#include <eros/SystemMonitor/Window/IWindow.h>
#include <eros/SystemMonitor/Window/WindowTable.h>
#include <eros/SystemMonitor/Window/WindowText.h>
#include <gtest/gtest.h>
#include <stdio.h>
using namespace eros;
class TesterIWindow : public IWindow
{
   public:
    WindowSize getWindowSize() {
        WindowSize size;
        size.min_height_pixel = 0;
        size.min_width_pixel = 0;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        return records;
    }
    bool setRecords(std::vector<std::shared_ptr<IRecord>> newRecords) {
        records = newRecords;
        return true;
    }
    bool keyPressed(KeyMap key) {
        (void)key;
        return false;
    }
    WindowType getWindowType() {
        return IWindow::WindowType::INFO;
    }
    bool update(double currentTime_s) {
        (void)currentTime_s;
        return true;
    }
    void setActualWindowSize(ScreenCoordinatePixel size) {
        (void)size;
    }

   private:
    std::vector<std::shared_ptr<IRecord>> records;
};
TEST(BasicTest, IWindowTest) {
    TesterIWindow window;
    EXPECT_EQ(window.getWindowSize().min_height_pixel, 0);
    EXPECT_EQ(window.getWindowSize().min_width_pixel, 0);
    eros::KeyMap invalidKey;
    EXPECT_FALSE(window.keyPressed(invalidKey));

    std::vector<std::shared_ptr<IRecord>> records;
    std::shared_ptr<GenericRecord> record(new GenericRecord);
    std::vector<std::shared_ptr<IField>> fields;
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "1";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "2";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    {
        std::shared_ptr<GenericField> field(new GenericField);
        RenderData data;
        data.data = "3";
        field->setData(data);
        fields.push_back(std::move(field));
    }
    record->setFields(fields);
    EXPECT_EQ(fields.size(), record->getFields().size());
    EXPECT_GT(fields.size(), 0);
    uint16_t counter = 1;
    for (auto field : record->getFields()) {
        uint16_t v = std::stoi(field->getData().data);
        EXPECT_EQ(v, counter);
        counter++;
    }
    records.push_back(std::move(record));
    EXPECT_EQ(records.size(), 1);
    EXPECT_TRUE(window.setRecords(records));
    EXPECT_EQ(records.size(), window.getRecords().size());
}
class BaseWindowTester : public BaseWindow
{
   public:
    BaseWindowTester(eros::Logger* logger) : BaseWindow(logger, IWindow::WindowType::INFO) {
    }
    WindowSize getWindowSize() {
        WindowSize size;
        size.min_height_pixel = 0;
        size.min_width_pixel = 0;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        return records;
    }
    bool setRecords(std::vector<std::shared_ptr<IRecord>> newRecords) {
        records = newRecords;
        return true;
    }
    bool keyPressed(KeyMap key) {
        (void)key;
        return false;
    }

   private:
    std::vector<std::shared_ptr<IRecord>> records;
};
TEST(BasicTest, SupportFunctions) {
    for (uint8_t i = 0; i <= (uint8_t)Level::Type::END_OF_LIST; ++i) {
        Color color = BaseWindow::convertLevelToColor((Level::Type)i);
        EXPECT_NE(color, Color::END_OF_LIST);
        EXPECT_NE(color, Color::UNKNOWN);
    }
    for (uint8_t i = 0; i <= (uint8_t)Node::State::END_OF_LIST; ++i) {
        Color color = BaseWindow::convertNodeStateToColor((Node::State)i);
        EXPECT_NE(color, Color::END_OF_LIST);
        EXPECT_NE(color, Color::UNKNOWN);
    }
}
TEST(BasicTest, BaseWindowTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    BaseWindowTester window(logger);
    EXPECT_EQ(window.getWindowType(), IWindow::WindowType::INFO);
    EXPECT_TRUE(window.update(123.0));
    delete logger;
}
class WindowTableTester : public WindowTable
{
   public:
    WindowTableTester(eros::Logger* logger) : WindowTable(logger, IWindow::WindowType::DEVICE) {
    }
    WindowSize getWindowSize() {
        WindowSize size;
        size.min_height_pixel = 0;
        size.min_width_pixel = 0;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        return records;
    }
    bool setRecords(std::vector<std::shared_ptr<IRecord>> newRecords) {
        records = newRecords;
        recordCount = records.size();
        return true;
    }
    bool keyPressed(KeyMap key) {
        return WindowTable::keyPressed(key);
    }

   private:
    std::vector<std::shared_ptr<IRecord>> records;
};
TEST(BasicTest, WindowTableTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    WindowTableTester SUT(logger);
    EXPECT_EQ(SUT.getRecords().size(), 0);
    {
        EXPECT_TRUE(SUT.keyPressed(KeyMap::KEY_None));
        EXPECT_TRUE(SUT.keyPressed(KeyMap::KEY_up));
        EXPECT_TRUE(SUT.keyPressed(KeyMap::KEY_down));
    }

    {
        std::vector<WindowTable::ColumnLabel> labels;
        SUT.setColumnLabels(labels);
    }
    // Add a record
    {
        std::vector<std::shared_ptr<IRecord>> records;
        std::shared_ptr<GenericRecord> record(new GenericRecord);
        std::vector<std::shared_ptr<IField>> fields;
        std::shared_ptr<GenericField> field(new GenericField);
        fields.push_back(std::move(field));
        record->setFields(fields);
        records.push_back(std::move(record));
        EXPECT_TRUE(SUT.setRecords(records));
    }
    for (int i = 0; i < 5; ++i) { EXPECT_TRUE(SUT.keyPressed(KeyMap::KEY_up)); }
    for (int i = 0; i < 5; ++i) { EXPECT_TRUE(SUT.keyPressed(KeyMap::KEY_down)); }
    delete logger;
}
class WindowTextTester : public WindowText
{
   public:
    WindowTextTester(eros::Logger* logger) : WindowText(logger, IWindow::WindowType::INFO) {
    }
    WindowSize getWindowSize() {
        WindowSize size;
        size.min_height_pixel = 0;
        size.min_width_pixel = 0;
        return size;
    }
    std::vector<std::shared_ptr<IRecord>> getRecords() {
        return records;
    }
    bool setRecords(std::vector<std::shared_ptr<IRecord>> newRecords) {
        records = newRecords;
        return true;
    }
    bool keyPressed(KeyMap key) {
        (void)key;
        return false;
    }

   private:
    std::vector<std::shared_ptr<IRecord>> records;
};
TEST(BasicTest, WindowTextTest) {
    Logger* logger = new Logger("DEBUG", "UnitTestSystemMonitor");
    WindowTextTester window(logger);
    delete logger;
}
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
