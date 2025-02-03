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
    std::vector<std::string> instruction_string;
    // Instructions that are always supported
    instruction_string.push_back("Space: Reset Screen");
    instruction_string.push_back("S: Start System Snapshot. (C: Clear Snapshots)");
    if (diagnostic_mode == DiagnosticMode::SYSTEM) {
        instruction_string.push_back("D: View NODE Diagnostics");
    }
    else if (diagnostic_mode == DiagnosticMode::NODE) {
        instruction_string.push_back("D: View SYSTEM Diagnostics");
    }
    if (instruction_mode == InstructionMode::NODE) {
        instruction_string.push_back("F: Get Node Firmware.");
        instruction_string.push_back("L: Change Log Level.");
        instruction_string.push_back("N: Change Node State (1-9).");
        /*
        if (show_task_diagnostic_mode == false) {
            instruction_string.push_back("D: Show Task Diagnostics.");
        }
        else {
            instruction_string.push_back("D: Show System Diagnostics.");
        }
        */
    }

    else {
        logger->log_warn("Mode: " + std::to_string((uint8_t)instruction_mode) + " Not Supported!");
        return false;
    }
    for (std::size_t i = 0; i < instruction_string.size(); ++i) {
        mvwprintw(get_window(), i + 3, 1, instruction_string.at(i).c_str());
        wclrtoeol(get_window());
    }
    wclrtobot(get_window());
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return true;
}
KeyEventContainer InstructionWindow::new_keyevent(int key) {
    KeyEventContainer output;
    // Keys that are always supported.
    if ((key == KEY_space)) {
        output.command.type = WindowCommandType::VIEW_DIAGNOSTICS_SYSTEM;
        std::string str = "Requesting Diagnostics for System";
        MessageText message(str, eros::Level::Type::INFO);
        logger->log_debug(str);
        output.message = message;
        return output;
    }
    else if ((key == KEY_s) || (key == KEY_S)) {
        MessageText message("Requesting System Snapshot...", eros::Level::Type::INFO);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::RUN_MASTER;
        command_pub.publish(command);
        output.message = message;
        return output;
    }
    else if ((key == KEY_c) || (key == KEY_C)) {
        MessageText message("Clearing All Snapshots...", eros::Level::Type::WARN);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)eros::Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)eros::Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
        command_pub.publish(command);
        output.message = message;
        return output;
    }

    // Specific Key/Mode support
    if (instruction_mode == InstructionMode::NODE) {}
    else {
        logger->log_warn("Mode: " + std::to_string((uint8_t)instruction_mode) + " Not Supported!");
        return output;
    }
    return output;
}
}  // namespace eros_nodes::SystemMonitor