#include "MasterNodeProcess.h"

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
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::new_commandmsg(
    const eros::command::ConstPtr& t_msg) {
    (void)t_msg;  // Currently Unused
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> MasterNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
Architecture::Type MasterNodeProcess::read_device_architecture() {
    // Try 1: Read command: "uname -m"
    {
        std::string cmd = "uname -m";
        std::string result = exec(cmd.c_str(), true);
        std::size_t found_x86_64 = result.find("x86_64");
        std::size_t found_armv7l = result.find("armv7l");
        std::size_t found_aarch64 = result.find("aarch64");
        if (found_x86_64 != std::string::npos) {
            return Architecture::Type::X86_64;
        }
        else if (found_armv7l != std::string::npos) {
            return Architecture::Type::ARMV7L;
        }
        else if (found_aarch64 != std::string::npos) {
            return Architecture::Type::AARCH64;
        }
        else {
            logger->log_warn("Unexpected result of command: " + cmd + " result: " + result);
        }
    }
    return Architecture::Type::UNKNOWN;
}