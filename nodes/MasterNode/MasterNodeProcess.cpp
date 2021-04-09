#include <eros/MasterNode/MasterNodeProcess.h>

MasterNodeProcess::MasterNodeProcess() {
}
MasterNodeProcess::~MasterNodeProcess() {
}
Diagnostic::DiagnosticDefinition MasterNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void MasterNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition MasterNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = convert(diag);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Currently Unused
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
