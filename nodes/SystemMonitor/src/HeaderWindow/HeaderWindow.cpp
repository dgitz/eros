#include "HeaderWindow/HeaderWindow.h"
namespace eros_nodes::SystemMonitor {
HeaderWindow::~HeaderWindow() {
}
bool HeaderWindow::new_msg(eros::ArmDisarm::State armed_state) {
    armed_state_ = armed_state;
    return true;
}
bool HeaderWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    status = update_window();
    return status;
}
bool HeaderWindow::update_window() {
    {  // Armed State

        Color color;
        std::string str = "Armed State: " + eros::ArmDisarm::ArmDisarmString(armed_state_.state);
        str.insert(str.end(), 40 - str.size(), ' ');
        switch (armed_state_.state) {
            case eros::ArmDisarm::Type::ARMED:
                color = Color::GREEN_COLOR;
                break;  // Should be BLUE for RC Mode, GREEN for Manual, PURPLE for Auto
            case eros::ArmDisarm::Type::DISARMED_CANNOTARM: color = Color::RED_COLOR; break;
            case eros::ArmDisarm::Type::DISARMED: color = Color::GREEN_COLOR; break;
            case eros::ArmDisarm::Type::DISARMING: color = Color::GREEN_COLOR; break;
            case eros::ArmDisarm::Type::ARMING: color = Color::GREEN_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }
        wattron(get_window(), COLOR_PAIR(color));
        mvwprintw(get_window(), 2, 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
    }

    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
}
}  // namespace eros_nodes::SystemMonitor