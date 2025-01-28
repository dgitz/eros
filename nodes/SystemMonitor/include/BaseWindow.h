#pragma once
#include <curses.h>
#include <eros/Diagnostic.h>
#include <eros/Logger.h>
#include <eros/eROS_Definitions.h>
#include <eros/heartbeat.h>

#include <string>

#include "IWindow.h"
#include "SystemMonitorUtilities.h"
#include "Window_Definitions.h"
namespace eros_nodes::SystemMonitor {

class BaseWindow : public IWindow
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

   protected:
    std::string name;
    ScreenCoordinatePerc screen_coord_perc;
    ScreenCoordinatePixel screen_coord_pixel;
    WINDOW* win_;
    eros::Logger* logger;
    uint16_t mainwindow_height;
    uint16_t mainwindow_width;

    double t_ros_time_{0.0};
    eros::Diagnostic::DiagnosticDefinition update(double /*dt*/, double t_ros_time);

    bool active{false};

    eros::Diagnostic::DiagnosticDefinition root_diagnostic;

   private:
};
}  // namespace eros_nodes::SystemMonitor