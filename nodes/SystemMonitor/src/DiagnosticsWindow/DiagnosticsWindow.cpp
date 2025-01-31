#include "DiagnosticsWindow/DiagnosticsWindow.h"
namespace eros_nodes::SystemMonitor {
DiagnosticsWindow::~DiagnosticsWindow() {
}

bool DiagnosticsWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool DiagnosticsWindow::update_window() {
    if (focused) {
        box(get_window(), '+', '+');
    }
    else {
        box(get_window(), 0, 0);
    }
    wrefresh(get_window());
    return true;
}
}  // namespace eros_nodes::SystemMonitor