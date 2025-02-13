/*! \file test_diagnosticswindow.cpp
 */
#include <gtest/gtest.h>
#include <stdio.h>

#include "DiagnosticsWindow/DiagnosticsWindow.h"
#include "WindowDefinitions.h"
using namespace eros_nodes::SystemMonitor;
TEST(BasicTest, Test_Initialization) {
    eros::Logger* logger = new eros::Logger("INFO", "test_diagnostics_window");
    EXPECT_TRUE(false);
    /*
    DeviceWindow SUT(nullptr, "/", logger, 0, 400, 400);
    // Verify Properties
    EXPECT_EQ(SUT.get_name(), "device_window");
    EXPECT_FALSE(SUT.has_focus());
    SUT.set_focused(false);
    ScreenCoordinatePixel empty_coordinates_pixel(0.0, 0.0, 0.0, 0.0);
    SUT.set_screen_coordinates_pix(empty_coordinates_pixel);
    auto screen_coord_perc = SUT.get_screen_coordinates_perc();
    EXPECT_EQ(screen_coord_perc.start_x_perc, DeviceWindow::START_X_PERC);
    EXPECT_EQ(screen_coord_perc.start_y_perc, DeviceWindow::START_Y_PERC);
    EXPECT_EQ(screen_coord_perc.width_perc, DeviceWindow::WIDTH_PERC);
    EXPECT_EQ(screen_coord_perc.height_perc, DeviceWindow::HEIGHT_PERC);
    auto screen_coord_pixel = SUT.get_screen_coordinates_pixel();
    EXPECT_EQ(screen_coord_pixel.start_x_pix, 0);
    EXPECT_EQ(screen_coord_pixel.start_y_pix, 0);
    EXPECT_EQ(screen_coord_pixel.width_pix, 0);
    EXPECT_EQ(screen_coord_pixel.height_pix, 0);
    logger->log_debug(SUT.pretty());
    EXPECT_EQ(SUT.get_tab_order(), 0);
    SUT.set_window_records_are_selectable(false);
    EXPECT_FALSE(SUT.get_window_records_are_selectable());
    EXPECT_EQ(SUT.get_selected_record(), 0);
    EXPECT_TRUE(SUT.is_selectable());

    // Verify Unsupported Commands
    {
        eros::ArmDisarm::State msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::heartbeat msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        eros::command_state msg;
        EXPECT_TRUE(SUT.new_msg(msg));
    }
    {
        int key = -1;
        SUT.new_keyevent(key);
    }
    {
        std::vector<WindowCommand> commands;
        EXPECT_TRUE(SUT.new_command(commands));
    }
    // Verify Events
    auto output = SUT.new_keyevent(KEY_UP);
    output = SUT.new_keyevent(KEY_DOWN);
    eros::resource resource_msg;
    EXPECT_FALSE(SUT.new_msg(resource_msg));

    output = SUT.new_keyevent(KEY_5);
    EXPECT_FALSE(SUT.update(0.0, 0.0));  // Can't update Window, this requires Drawing.
    */
    delete logger;
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}