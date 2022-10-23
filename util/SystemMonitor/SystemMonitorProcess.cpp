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
        HeaderWindow* window = new HeaderWindow();
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::HEADER, window));
    }
    {
        ProcessWindow* window = new ProcessWindow();
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::PROCESS, window));
    }
    {
        NodeDiagnosticsWindow* window = new NodeDiagnosticsWindow();
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::NODEDIAGNOSTICS, window));
    }
    {
        InfoWindow* window = new InfoWindow();
        windows.insert(std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::INFO, window));
    }
    {
        DeviceWindow* window = new DeviceWindow();
        windows.insert(
            std::pair<IWindow::WindowType, IWindow*>(IWindow::WindowType::DEVICE, window));
    }
    return true;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::new_heartbeatmessage(
    const eros::heartbeat::ConstPtr& t_msg) {
    eros::heartbeat msg = convert_fromptr(t_msg);
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
