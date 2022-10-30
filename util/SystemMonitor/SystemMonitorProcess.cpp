// No practical way to this file due to screen rendering.
// LCOV_EXCL_START
#include <eros/SystemMonitor/SystemMonitorProcess.h>
using namespace eros;
SystemMonitorProcess::SystemMonitorProcess() {
}
SystemMonitorProcess::~SystemMonitorProcess() {
    for (std::map<IWindow::WindowType, IWindow*>::iterator it = windows.begin();
         it != windows.end();
         it++) {
        delete it->second;
    }
    windows.clear();
    delete renderEngine;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::finish_initialization() {
    logger->disable_consoleprint();
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    bool status = initializeWindows();
    if (status == false) {
        diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                 Level::Type::ERROR,
                                 Diagnostic::Message::INITIALIZING_ERROR,
                                 "Unable to Initialize Windows.");
        logger->enable_consoleprint();
        logger->log_diagnostic(diag);
    }
    renderEngine = new RenderEngine(logger, windows);
    if (renderEngine->initScreen() == false) {
        diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                 Level::Type::ERROR,
                                 Diagnostic::Message::INITIALIZING_ERROR,
                                 "Unable to Initialize Screen.");
        logger->enable_consoleprint();
        logger->log_diagnostic(diag);
    }
    return diag;
}
void SystemMonitorProcess::reset() {
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    for (auto win : windows) {
        bool v = win.second->update(t_ros_time);
        if (v == false) {
            logger->log_warn("Unable to Update Window: " + std::to_string((uint8_t)win.first));
        }
    }
    renderEngine->update(t_dt, windows);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
bool SystemMonitorProcess::initializeWindows() {
    {
        StatusWindow* window = new StatusWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(window->getWindowType(), window));
    }
    {
        ProcessWindow* window = new ProcessWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(window->getWindowType(), window));
    }
    {
        NodeDiagnosticsWindow* window = new NodeDiagnosticsWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(window->getWindowType(), window));
    }
    {
        InfoWindow* window = new InfoWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(window->getWindowType(), window));
    }
    {
        DeviceWindow* window = new DeviceWindow(logger);
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(window->getWindowType(), window));
    }
    return true;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_heartbeatmessage(
    const eros::heartbeat::ConstPtr& t_msg) {
    eros::heartbeat msg = convert_fromptr(t_msg);
    return new_heartbeatmessage(msg);
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_genericNode(std::string hostName, std::string nodeName, double currentTime_s) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window.first == IWindow::WindowType::PROCESS) {
            ProcessWindow* win = dynamic_cast<ProcessWindow*>(window.second);
            bool v = win->new_nodeAlive(hostName, nodeName, currentTime_s);
            if (v == false) {
                diag = update_diagnostic(
                    Diagnostic::DiagnosticType::COMMUNICATIONS,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to Update Window: " + std::to_string((uint8_t)window.first));
                return diag;
            }
        }
    }
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_heartbeatmessage(eros::heartbeat msg) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window.first == IWindow::WindowType::PROCESS) {
            ProcessWindow* win = dynamic_cast<ProcessWindow*>(window.second);
            bool v = win->new_heartbeat(msg);
            if (v == false) {
                diag = update_diagnostic(
                    Diagnostic::DiagnosticType::COMMUNICATIONS,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to Update Window: " + std::to_string((uint8_t)window.first));
                return diag;
            }
        }
    }
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceusedmessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = convert_fromptr(t_msg);
    return new_resourceusedmessage(msg);
}

Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceusedmessage(eros::resource msg) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window.first == IWindow::WindowType::PROCESS) {
            ProcessWindow* win = dynamic_cast<ProcessWindow*>(window.second);
            bool v = win->new_resource(msg);
            if (v == false) {
                diag = update_diagnostic(
                    Diagnostic::DiagnosticType::COMMUNICATIONS,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to Update Window: " + std::to_string((uint8_t)window.first));
                return diag;
            }
        }
    }
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceavailablemessage(
    const eros::resource::ConstPtr& t_msg) {
    eros::resource msg = convert_fromptr(t_msg);
    return new_resourceavailablemessage(msg);
}

Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_resourceavailablemessage(
    eros::resource msg) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window.first == IWindow::WindowType::DEVICE) {
            DeviceWindow* win = dynamic_cast<DeviceWindow*>(window.second);
            bool v = win->new_resourceavailable(msg);
            if (v == false) {
                diag = update_diagnostic(
                    Diagnostic::DiagnosticType::COMMUNICATIONS,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to Update Window: " + std::to_string((uint8_t)window.first));
                return diag;
            }
        }
    }
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_loadfactormessage(
    const eros::loadfactor::ConstPtr& t_msg) {
    eros::loadfactor msg = convert_fromptr(t_msg);
    return new_loadfactormessage(msg);
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_loadfactormessage(eros::loadfactor msg) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    for (auto window : windows) {
        if (window.first == IWindow::WindowType::DEVICE) {
            DeviceWindow* win = dynamic_cast<DeviceWindow*>(window.second);
            bool v = win->new_loadfactor(msg);
            if (v == false) {
                diag = update_diagnostic(
                    Diagnostic::DiagnosticType::COMMUNICATIONS,
                    Level::Type::ERROR,
                    Diagnostic::Message::DROPPING_PACKETS,
                    "Unable to Update Window: " + std::to_string((uint8_t)window.first));
                return diag;
            }
        }
    }
    return diag;
}
// LCOV_EXCL_STOP