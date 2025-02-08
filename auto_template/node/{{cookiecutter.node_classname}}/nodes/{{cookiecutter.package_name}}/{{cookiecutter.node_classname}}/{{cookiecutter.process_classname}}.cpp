#include <{{cookiecutter.package_name}}/{{cookiecutter.node_classname}}/{{cookiecutter.process_classname}}.h>
using namespace eros;
{{cookiecutter.process_classname}}::{{cookiecutter.process_classname}}() {
}
{{cookiecutter.process_classname}}::~{{cookiecutter.process_classname}}() {
}
eros_diagnostic::Diagnostic {{cookiecutter.process_classname}}::finish_initialization() {
    eros_diagnostic::Diagnostic diag = get_root_diagnostic();
    return diag;
}
void {{cookiecutter.process_classname}}::reset() {
}
eros_diagnostic::Diagnostic {{cookiecutter.process_classname}}::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<eros_diagnostic::Diagnostic> {{cookiecutter.process_classname}}::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Command Messages Supported at this time.");
    return diag_list;
}
std::vector<eros_diagnostic::Diagnostic> {{cookiecutter.process_classname}}::check_programvariables() {
    std::vector<eros_diagnostic::Diagnostic> diag_list;
    logger->log_warn("No Program Variables Checked.");
    return diag_list;
}
