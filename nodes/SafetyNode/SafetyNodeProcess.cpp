#include "SafetyNodeProcess.h"

SafetyNodeProcess::SafetyNodeProcess() {
    armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;  // No monitors defined yet.
}
SafetyNodeProcess::~SafetyNodeProcess() {
}
Diagnostic::DiagnosticDefinition SafetyNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void SafetyNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition SafetyNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SafetyNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SafetyNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
