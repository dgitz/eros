#include "SafetyNodeProcess.h"
using namespace eros;
using namespace eros_nodes;

SafetyNodeProcess::SafetyNodeProcess() {
}
SafetyNodeProcess::~SafetyNodeProcess() {
}
eros_diagnostic::Diagnostic SafetyNodeProcess::finish_initialization() {
    eros_diagnostic::Diagnostic diag;
    diag = update_diagnostic(eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                             Level::Type::INFO,
                             eros_diagnostic::Message::NODATA,
                             "No Remote Control Command Yet.");
    return diag;
}
void SafetyNodeProcess::reset() {
}
eros_diagnostic::Diagnostic SafetyNodeProcess::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> SafetyNodeProcess::new_commandmsg(eros::command msg) {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> SafetyNodeProcess::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    return diag_list;
}
bool SafetyNodeProcess::new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm) {
    return false;
}