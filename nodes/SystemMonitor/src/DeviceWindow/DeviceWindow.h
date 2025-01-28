#pragma once
#include "../BaseWindow.h"
namespace eros_nodes::SystemMonitor {
class DeviceWindow : public BaseWindow
{
   public:
    DeviceWindow(eros::Logger* logger, uint16_t mainwindow_height, uint16_t mainwindow_width)
        : BaseWindow("device_window",
                     55.0,
                     80.0,
                     45.0,
                     20.0,
                     logger,
                     mainwindow_height,
                     mainwindow_width) {
        logger->log_warn("Device Header");
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
    virtual ~DeviceWindow();
    eros::Diagnostic::DiagnosticDefinition update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State armed_state) override;
    bool new_msg(eros::heartbeat heartbeat_msg) override {
        return true;
    }

   private:
};
}  // namespace eros_nodes::SystemMonitor