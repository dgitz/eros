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
    std::vector<std::string> resourceused_list,
    std::vector<std::string> resourceavailable_list,
    std::vector<std::string> loadfactor_list,
    std::vector<std::string>& new_heartbeat_topics_to_subscribe,
    std::vector<std::string>& new_resourceused_topics_to_subscribe,
    std::vector<std::string>& new_resourceavailable_topics_to_subscribe,
    std::vector<std::string>& new_loadfactor_topics_to_subscribe) {
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
    // Check for new Resource Used messages
    for (auto resourceused : resourceused_list) {
        bool found_it = false;
        for (auto monitored_resourceused : monitored_resourceused_topics) {
            if (monitored_resourceused == resourceused) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_resourceused_topics.push_back(resourceused);
            new_resourceused_topics_to_subscribe.push_back(resourceused);
        }
    }
    // Check for new Resource Available messages
    for (auto resourceavailable : resourceavailable_list) {
        bool found_it = false;
        for (auto monitored_resourceavailable : monitored_resourceavailable_topics) {
            if (monitored_resourceavailable == resourceavailable) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_resourceavailable_topics.push_back(resourceavailable);
            new_resourceavailable_topics_to_subscribe.push_back(resourceavailable);
        }
    }
    // Check for new LoadFactor messages
    for (auto loadfactor : loadfactor_list) {
        bool found_it = false;
        for (auto monitored_loadfactor : monitored_loadfactor_topics) {
            if (monitored_loadfactor == loadfactor) {
                found_it = true;
                break;
            }
        }
        if (found_it == false) {
            monitored_loadfactor_topics.push_back(loadfactor);
            new_loadfactor_topics_to_subscribe.push_back(loadfactor);
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
    for (auto window : windows) {
        if (window->has_focus()) {
            bool status = window->new_keyevent(key_pressed);
            if (status == false) {
                diag = diagnostic_helper.update_diagnostic(
                    Diagnostic::DiagnosticType::SOFTWARE,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to update Window: " + window->get_name() + " With new Key Event.");
            }
        }
        window->update(t_dt, t_ros_time);
    }
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
        window->set_focused(true);  // Window defaults to focused
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
    {
        IWindow* window = new InstructionWindow(logger, mainwindow_height, mainwindow_width);
        window->set_focused(true);  // Window always has focus
        windows.push_back(window);
    }
    {
        IWindow* window = new DiagnosticsWindow(logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        IWindow* window = new MessageWindow(logger, mainwindow_height, mainwindow_width);
        windows.push_back(window);
    }
    {
        IWindow* window = new StatusWindow(logger, mainwindow_height, mainwindow_width);
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
                "Unable to update Window: " + window->get_name() + " With new Heartbeat.");
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
                "Unable to update Window: " + window->get_name() + " With new Armed State.");
        }
    }
}
eros::Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceusedmessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = convert_fromptr(t_msg);
    eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window->get_name() != "device_window") {
            bool status = window->new_msg(msg);
            if (status == false) {
                diag = diagnostic_helper.update_diagnostic(
                    Diagnostic::DiagnosticType::SOFTWARE,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to update Window: " + window->get_name() + " With new Resource Used.");
            }
        }
    }
    return diag;
}
eros::Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_loadfactormessage(
    const eros::loadfactor::ConstPtr& t_msg) {
    eros::loadfactor msg = convert_fromptr(t_msg);
    eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        bool status = window->new_msg(msg);
        if (status == false) {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::SOFTWARE,
                Level::Type::ERROR,
                Diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Window: " + window->get_name() + " With new Load Factor Data.");
        }
    }
    return diag;
}
eros::Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceavailablemessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = convert_fromptr(t_msg);
    eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window->get_name() == "device_window") {
            bool status = window->new_msg(msg);
            if (status == false) {
                diag = diagnostic_helper.update_diagnostic(
                    Diagnostic::DiagnosticType::SOFTWARE,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to update Window: " + window->get_name() +
                        " With new Resource Available.");
            }
        }
    }
    return diag;
}
std::string SystemMonitorProcess::pretty() {
    std::string str = "\n----- System Monitor Node Process -----\n";
    str += "Monitored Topics:Heartbeat(" +
           std::to_string((uint16_t)monitored_heartbeat_topics.size()) + ")\n";
    for (auto v : monitored_heartbeat_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Resource Used(" +
           std::to_string((uint16_t)monitored_resourceused_topics.size()) + ")\n";
    for (auto v : monitored_resourceused_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Resource Available(" +
           std::to_string((uint16_t)monitored_resourceavailable_topics.size()) + ")\n";
    for (auto v : monitored_resourceavailable_topics) { str += "\t" + v + "\n"; }
    str += "Monitored Topics:Load Factor(" +
           std::to_string((uint16_t)monitored_loadfactor_topics.size()) + ")\n";
    for (auto v : monitored_loadfactor_topics) { str += "\t" + v + "\n"; }
    return str;
}
}  // namespace eros_nodes::SystemMonitor