#include "InstructionWindow/InstructionWindow.h"

#include <eros/command.h>
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
    bool select_task_mode = true;            // HACK
    bool change_log_level_mode = false;      // HACK
    bool show_task_diagnostic_mode = false;  // HACK
    bool change_nodestate_mode = false;      // HACK
    if (select_task_mode == true) {
        std::vector<std::string> instruction_string;
        instruction_string.push_back("S: Generate System Snapshot. (C: Clear Snapshots)");
        instruction_string.push_back("F: Get Node Firmware.");
        instruction_string.push_back("L: Change Log Level.");
        if (change_log_level_mode == true) {
            instruction_string.push_back("  1,2,3,4,5,6: Select Log Level.");
        }
        if (show_task_diagnostic_mode == false) {
            instruction_string.push_back("D: Show Task Diagnostics.");
        }
        else {
            instruction_string.push_back("D: Show System Diagnostics.");
        }
        instruction_string.push_back("N: Change Node State.");
        if (change_nodestate_mode == true) {
            instruction_string.push_back("  1-9: Select Node State.");
        }
        for (std::size_t i = 0; i < instruction_string.size(); ++i) {
            mvwprintw(win_, i + 3, 1, instruction_string.at(i).c_str());
            wclrtoeol(win_);
        }
    }
    wclrtobot(win_);
    box(win_, 0, 0);
    wrefresh(win_);
    return true;
}
MessageText InstructionWindow::new_keyevent(int key) {
    if ((key == KEY_s) || (key == KEY_S)) {
        MessageText message("Requesting System Snapshot...", eros::Level::Type::INFO);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER;
        command_pub.publish(command);
        return message;
    }
    else if ((key == KEY_c) || (key == KEY_C)) {
        MessageText message("Clearing All Snapshots...", eros::Level::Type::WARN);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
        command_pub.publish(command);
        return message;
    }
    MessageText empty;
    return empty;
}
}  // namespace eros_nodes::SystemMonitor