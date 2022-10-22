#include <eros/SystemMonitor/SystemMonitorProcess.h>
using namespace eros;
SystemMonitorProcess::SystemMonitorProcess() {
}
SystemMonitorProcess::~SystemMonitorProcess() {
    for (std::map<std::string, IWindow*>::iterator it = windows.begin(); it != windows.end();
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
        WindowTable* window = new WindowTable();
        windows.insert(std::pair<std::string, IWindow*>("dumb", window));
    }
    return true;
}