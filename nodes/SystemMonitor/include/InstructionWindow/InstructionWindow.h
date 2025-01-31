#pragma once
#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
class InstructionWindow : public BaseWindow
{
   public:
    InstructionWindow(ros::NodeHandle* nodeHandle,
                      std::string robot_namespace,
                      eros::Logger* logger,
                      uint16_t mainwindow_height,
                      uint16_t mainwindow_width,
                      ros::Publisher command_pub)
        : BaseWindow("instruction_window",
                     30,
                     80.0,
                     25.0,
                     20.0,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width),
          command_pub(command_pub) {
        ScreenCoordinatePixel coord_pix =
            convertCoordinate(get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = create_newwin(coord_pix.height_pix,
                                    coord_pix.width_pix,
                                    coord_pix.start_y_pix,
                                    coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);

        std::string str = "Instructions:";
        keypad(win, TRUE);
        mvwprintw(win, 1, 1, str.c_str());
        std::string dashed(coord_pix.width_pix - 2, '-');
        mvwprintw(win, 2, 1, dashed.c_str());
        wrefresh(win);
    }
    virtual ~InstructionWindow();
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
    MessageText new_keyevent(int key) override;

   private:
    bool update_window();
    ros::Publisher command_pub;
};
}  // namespace eros_nodes::SystemMonitor