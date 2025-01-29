#include "MessageWindow/MessageWindow.h"
namespace eros_nodes::SystemMonitor {
MessageWindow::~MessageWindow() {
}

bool MessageWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool MessageWindow::update_window() {
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
}
bool MessageWindow::new_MessageTextList(std::vector<MessageText> messages) {
    logger->log_error("Not Implemented Yet!");
    return false;
}
}  // namespace eros_nodes::SystemMonitor