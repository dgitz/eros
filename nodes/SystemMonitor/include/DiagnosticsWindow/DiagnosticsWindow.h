#pragma once
#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
class DiagnosticsWindow : public BaseWindow
{
   public:
    const double REQUEST_DATA_RATE = 1.0;  // Hz
    enum class DiagnosticMode { UNKNOWN = 0, SYSTEM = 1, NODE = 2 };
    DiagnosticsWindow(ros::NodeHandle* nodeHandle,
                      std::string robot_namespace,
                      eros::Logger* logger,
                      int16_t tab_order,
                      uint16_t mainwindow_height,
                      uint16_t mainwindow_width)
        : BaseWindow("diagnostics_window",
                     tab_order,
                     66,
                     15.0,
                     34.5,
                     60.0,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width) {
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
    virtual ~DiagnosticsWindow();
    bool is_selectable() override {
        return true;
    }
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State /* armed_state */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::heartbeat /* heartbeat_msg */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::resource /*resource_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::loadfactor /*loadfactor_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::command_state /* command_state_msg */) override {  // Not Used
        return true;
    }
    KeyEventContainer new_keyevent(int /* key */) override {  // Not Used
        KeyEventContainer output;
        return output;
    }
    bool new_command(std::vector<WindowCommand> commands) override;

   private:
    bool update_window();
    DiagnosticMode diagnostic_mode{DiagnosticMode::SYSTEM};
    std::map<uint8_t, eros::eros_diagnostic::Diagnostic> diagnostic_data;
    double request_data_timer{0.0};
    std::string node_to_monitor{""};
};
}  // namespace eros_nodes::SystemMonitor