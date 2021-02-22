#include "SnapshotProcess.h"

SnapshotProcess::SnapshotProcess()
    : mode(Mode::UNKNOWN), architecture(Architecture::Type::UNKNOWN) {
}
SnapshotProcess::~SnapshotProcess() {
}
Diagnostic::DiagnosticDefinition SnapshotProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    if (mode == Mode::UNKNOWN) {
        diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                   "Mode not set!");
    }
    if (architecture == Architecture::Type::UNKNOWN) {
        diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                                   Level::Type::ERROR,
                                                   Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                                   "Architecture not set!");
    }
    return diag;
}
void SnapshotProcess::reset() {
}
Diagnostic::DiagnosticDefinition SnapshotProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::new_commandmsg(
    const eros::command::ConstPtr& t_msg) {
    (void)t_msg;  // Not Used
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
Diagnostic::DiagnosticDefinition SnapshotProcess::load_config(std::string file_path) {
    logger->log_notice("Loading: " + file_path);
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    TiXmlDocument doc(file_path);
    bool configfile_loaded = doc.LoadFile();
    if (configfile_loaded == false) {
        diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                 Level::Type::ERROR,
                                 Diagnostic::Message::INITIALIZING_ERROR,
                                 "Unable to load: " + file_path);
        return diag;
    }
    diag = update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                             Level::Type::ERROR,
                             Diagnostic::Message::DIAGNOSTIC_FAILED,
                             "Not Implemented Yet.");

    return diag;
}
