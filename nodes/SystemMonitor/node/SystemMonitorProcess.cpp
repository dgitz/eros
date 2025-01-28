#include "SystemMonitorProcess.h"

using namespace eros;
namespace eros_nodes::SystemMonitor {
SystemMonitorProcess::~SystemMonitorProcess() {
    for (auto window : windows) { delete window; }
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Finished Initialization.");
    return diag;
}
void SystemMonitorProcess::reset() {
}
eros::Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_monitorlist(
    std::vector<std::string> heartbeat_list,
    std::vector<std::string>& new_heartbeat_topics_to_subscribe) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    // Check for new Heartbeat messages
    for (auto heartbeat : heartbeat_list) {
        bool found_it = false;
        for (auto monitored_heartbeat : monitored_heartbeat_topics) {
            if (monitored_heartbeat == heartbeat) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_heartbeat_topics.push_back(heartbeat);
            new_heartbeat_topics_to_subscribe.push_back(heartbeat);
        }
    }
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    int key_pressed = getch();
    if ((key_pressed == KEY_q) || (key_pressed == KEY_Q)) {
        kill_me = true;
    }
    for (auto window : windows) { window->update(t_dt, t_ros_time); }
    flushinp();

    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;  // Not currently used.
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Processed Command.");
    diag_list.push_back(diag);
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Program Variable Check Passed.");
    diag_list.push_back(diag);
    return diag_list;
}
bool SystemMonitorProcess::initialize_windows() {
    timeout(0);
    {
        IWindow* window = new HeaderWindow(logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        IWindow* window = new DeviceWindow(logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        IWindow* window = new NodeWindow(logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    return true;
}
eros::Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_heartbeatmessage(
    const eros::heartbeat::ConstPtr& t_msg) {
    eros::heartbeat msg = convert_fromptr(t_msg);
    eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(msg);
        if (status == false) {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                Diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name());
        }
    }
    return diag;
}
void SystemMonitorProcess::update_armedstate(eros::ArmDisarm::State armed_state) {
    eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(armed_state);
        if (status == false) {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                Diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name());
        }
    }
}
}  // namespace eros_nodes::SystemMonitor