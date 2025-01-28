#pragma once
#include <curses.h>
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <eros/heartbeat.h>

#include <string>

#include "Window_Definitions.h"
namespace eros_nodes::SystemMonitor {
WINDOW* create_newwin(int height, int width, int starty, int startx) {
    WINDOW* local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0); /* 0, 0 gives default characters
                           * for the vertical and horizontal
                           * lines			*/
    wrefresh(local_win);  /* Show that box 		*/

    return local_win;
}
/*! \struct Field
    \brief Field container, used for holding Field attributes.
    */
struct Field {
    Field(std::string _text, uint16_t _width) : text(_text), width(_width) {
    }
    std::string text;
    std::size_t width;
};
class BaseWindow
{
   public:
    BaseWindow(const std::string _name,
               double start_x_perc,
               double start_y_perc,
               double width_perc,
               double height_perc,
               eros::Logger* logger,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : name(_name),
          screen_coord_perc(start_x_perc, start_y_perc, width_perc, height_perc),
          screen_coord_pixel(0, 0, 0, 0),
          win_(nullptr),
          logger(logger),
          mainwindow_height(mainwindow_height),
          mainwindow_width(mainwindow_width) {
    }
    ScreenCoordinatePixel convertCoordinate(ScreenCoordinatePerc coord_perc,
                                            uint16_t width_pix,
                                            uint16_t height_pix);
    virtual ~BaseWindow() {
    }
    std::string get_name() {
        return name;
    }
    bool is_active() {
        return active;
    }
    void set_active(bool cmd_active) {
        active = cmd_active;
    }
    void set_screen_coordinates_pix(ScreenCoordinatePixel coord) {
        screen_coord_pixel = coord;
    }
    ScreenCoordinatePerc get_screen_coordinates_perc() {
        return screen_coord_perc;
    }
    ScreenCoordinatePixel get_screen_coordinates_pixel() {
        return screen_coord_pixel;
    }
    std::string pretty() {
        if (win_ == nullptr) {
            return name + " Is Uninitialized.";
        }
        else {
            char tempstr[128];
            sprintf(tempstr,
                    "%s (X:%d%%Y:%d%%W:%d%%H:%d%%)",
                    name.c_str(),
                    (uint16_t)screen_coord_perc.start_x_perc,
                    (uint16_t)screen_coord_perc.start_y_perc,
                    (uint16_t)screen_coord_perc.width_perc,
                    (uint16_t)screen_coord_perc.height_perc);
            return std::string(tempstr);
        }
    }
    void set_window(WINDOW* win) {
        win_ = win;
    }
    WINDOW* get_window() {
        return win_;
    }
    virtual eros::Diagnostic::DiagnosticDefinition update(double dt, double t_ros_time) = 0;
    virtual bool new_msg(eros::ArmDisarm::State armed_state) = 0;
    virtual bool new_msg(eros::heartbeat heartbeat_msg) = 0;

   protected:
    double t_ros_time_{0.0};
    eros::Logger* logger;
    bool active{false};
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    eros::Diagnostic::DiagnosticDefinition root_diagnostic;
    std::string name;
    ScreenCoordinatePerc screen_coord_perc;
    ScreenCoordinatePixel screen_coord_pixel;
    WINDOW* win_;
};
}  // namespace eros_nodes::SystemMonitor