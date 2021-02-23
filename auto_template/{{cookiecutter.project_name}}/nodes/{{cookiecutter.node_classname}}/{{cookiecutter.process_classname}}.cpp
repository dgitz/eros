#include "{{cookiecutter.process_classname}}.h"

{{cookiecutter.process_classname}}::{{cookiecutter.process_classname}}() {
}
{{cookiecutter.process_classname}}::~{{cookiecutter.process_classname}}() {
}
Diagnostic::DiagnosticDefinition {{cookiecutter.process_classname}}::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    return diag;
}
void {{cookiecutter.process_classname}}::reset() {
}
Diagnostic::DiagnosticDefinition {{cookiecutter.process_classname}}::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> {{cookiecutter.process_classname}}::new_commandmsg(
    eros::command msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> {{cookiecutter.process_classname}}::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
