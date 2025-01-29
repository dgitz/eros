#include "InstructionWindow/InstructionWindow.h"
namespace eros_nodes::SystemMonitor {
InstructionWindow::~InstructionWindow() {
}

bool InstructionWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool InstructionWindow::update_window() {
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
}
}  // namespace eros_nodes::SystemMonitor