#pragma once
#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
class HeaderWindow : public BaseWindow
{
   public:
    HeaderWindow(eros::Logger* logger, uint16_t mainwindow_height, uint16_t mainwindow_width)
        : BaseWindow(
              "header_window", 0.0, 0.0, 100.0, 15.0, logger, mainwindow_height, mainwindow_width) {
        logger->log_warn("Initialized Header");
        ScreenCoordinatePixel coord_pix =
            convertCoordinate(get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = create_newwin(coord_pix.height_pix,
                                    coord_pix.width_pix,
                                    coord_pix.start_y_pix,
                                    coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~HeaderWindow();
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State armed_state);
    bool new_msg(eros::heartbeat /* heartbeat_msg */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::resource /*resource_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::loadfactor /*loadfactor_msg*/) override {  // Not Used
        return true;
    }

   private:
    bool update_window();
    eros::ArmDisarm::State armed_state_;
};
}  // namespace eros_nodes::SystemMonitor