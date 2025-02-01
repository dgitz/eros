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
#include "ros/ros.h"
namespace eros_nodes::SystemMonitor {

class BaseWindow : public IWindow
{
   public:
    BaseWindow(const std::string _name,
               int16_t tab_order,
               double start_x_perc,
               double start_y_perc,
               double width_perc,
               double height_perc,
               ros::NodeHandle* nodeHandle,
               std::string robot_namespace,
               eros::Logger* logger,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : name(_name),
          tab_order(tab_order),
          screen_coord_perc(start_x_perc, start_y_perc, width_perc, height_perc),
          screen_coord_pixel(0, 0, 0, 0),
          nodeHandle(nodeHandle),
          robot_namespace(robot_namespace),
          logger(logger),
          mainwindow_height(mainwindow_height),
          mainwindow_width(mainwindow_width),
          win_(nullptr) {
    }

    virtual ~BaseWindow() {
    }
    std::string get_name() {
        return name;
    }
    bool has_focus() {
        return focused;
    }
    void set_focused(bool cmd_focus) {
        focused = cmd_focus;
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
    virtual bool update_window() = 0;
    int16_t get_tab_order() {
        return tab_order;
    }
    void set_window_records_are_selectable(bool cmd) {
        if (cmd == true) {
            records_are_selectable = true;
            record_selected = 0;
        }
        else {
            records_are_selectable = false;
            record_selected = 0;
        }
    }
    bool get_window_records_are_selectable() {
        return records_are_selectable;
    }

   protected:
    std::string name;
    int16_t tab_order;
    ScreenCoordinatePerc screen_coord_perc;
    ScreenCoordinatePixel screen_coord_pixel;

    ros::NodeHandle* nodeHandle;
    std::string robot_namespace;
    eros::Logger* logger;
    uint16_t mainwindow_height;
    uint16_t mainwindow_width;

    double t_ros_time_{0.0};
    bool update(double /*dt*/, double t_ros_time);

    bool focused{false};

   private:
    WINDOW* win_;
    bool records_are_selectable{false};
    int16_t record_selected{-1};
};
}  // namespace eros_nodes::SystemMonitor