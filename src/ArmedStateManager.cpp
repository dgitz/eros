#include <eros/ArmedStateManager.h>

namespace eros {
ArmedStateManager::ArmedStateManager(std::string device_name,
                                     std::string node_name,
                                     System::MainSystem system,
                                     System::SubSystem sub_system)
    : current_diagnostic(device_name,
                         node_name,
                         system,
                         sub_system,
                         System::Component::CONTROLLER,
                         eros_diagnostic::DiagnosticType::REMOTE_CONTROL,
                         eros_diagnostic::Message::INITIALIZING,
                         Level::Type::WARN,
                         "Initializing") {
    current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
}
ArmedStateManager::~ArmedStateManager() {
}
eros_diagnostic::Diagnostic ArmedStateManager::update(double t_dt, double t_ros_time) {
    eros_diagnostic::Diagnostic diag = current_diagnostic;
    if (diag.message == eros_diagnostic::Message::INITIALIZING) {
        diag.message = eros_diagnostic::Message::NOERROR;
        diag.level = Level::Type::INFO;
        diag.description = "Updated";
    }
    current_diagnostic = diag;
    return diag;
}
bool ArmedStateManager::new_command(eros::command cmd) {
    return false;
}

bool ArmedStateManager::new_all_ready_to_arm(bool data) {
    return false;
}
bool ArmedStateManager::reset() {
    return false;
}
std::string ArmedStateManager::pretty() {
    std::string str = "-----Armed State Manager-----\n";
    str += "\tArmed State: " + ArmDisarm::ArmDisarmString(current_armed_state.state) + "\n";
    str += "\tDiag: " + eros_diagnostic::DiagnosticUtility::pretty("", current_diagnostic, false) +
           "\n";
    return str;
}
}  // namespace eros