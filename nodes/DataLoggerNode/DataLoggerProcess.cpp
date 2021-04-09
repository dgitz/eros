#include <eros/DataLogger/DataLoggerProcess.h>

DataLoggerProcess::DataLoggerProcess()
    : log_directory(""),
      log_directory_available(false),
      logfile_duration(-1.0),
      logging_enabled(false),
      snapshot_mode(true) {
}

DataLoggerProcess::~DataLoggerProcess() {
}
Diagnostic::DiagnosticDefinition DataLoggerProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void DataLoggerProcess::reset() {
}
Diagnostic::DiagnosticDefinition DataLoggerProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = convert(diag);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> DataLoggerProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Not used yet.
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> DataLoggerProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
