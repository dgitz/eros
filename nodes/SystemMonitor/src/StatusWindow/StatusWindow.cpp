#include "StatusWindow/StatusWindow.h"
namespace eros_nodes::SystemMonitor {
StatusWindow::~StatusWindow() {
}

bool StatusWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool StatusWindow::update_window() {
    if (get_window() == nullptr) {
        return false;
    }
    // GCOVR_EXCL_START
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
    // GCOVR_EXCL_STOP
}
}  // namespace eros_nodes::SystemMonitor