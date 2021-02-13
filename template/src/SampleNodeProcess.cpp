#include <test_node/SampleNodeProcess.h>

SampleNodeProcess::SampleNodeProcess() {
}
SampleNodeProcess::~SampleNodeProcess() {
}
Diagnostic::DiagnosticDefinition SampleNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void SampleNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition SampleNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SampleNodeProcess::new_commandmsg(
    const eros::command::ConstPtr& t_msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SampleNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}