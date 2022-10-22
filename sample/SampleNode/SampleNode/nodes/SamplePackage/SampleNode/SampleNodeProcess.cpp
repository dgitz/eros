#include <SamplePackage/SampleNode/SampleNodeProcess.h>
using namespace eros;
SampleNodeProcess::SampleNodeProcess() {
}
SampleNodeProcess::~SampleNodeProcess() {
}
Diagnostic::DiagnosticDefinition SampleNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    return diag;
}
void SampleNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition SampleNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SampleNodeProcess::new_commandmsg(eros::command msg) {
    (void)msg;
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SampleNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
