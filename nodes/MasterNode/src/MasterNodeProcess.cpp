#include "MasterNodeProcess.h"
using namespace eros;
using namespace eros_nodes;
MasterNodeProcess::MasterNodeProcess() {
}
MasterNodeProcess::~MasterNodeProcess() {
}
Diagnostic::DiagnosticDefinition MasterNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    return diag;
}
void MasterNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition MasterNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = eros::convert(diag);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;  // Currently Unused
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
