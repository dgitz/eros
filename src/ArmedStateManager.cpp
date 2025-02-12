#include <eros/ArmedStateManager.h>

namespace eros {
ArmedStateManager::ArmedStateManager(std::string device_name,
                                     std::string node_name,
                                     System::MainSystem system,
                                     System::SubSystem sub_system,
                                     std::vector<std::string> ready_to_arm_list)
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
    for (auto signal_name : ready_to_arm_list) {
        ReadyToArmSignal signal(signal_name);
        ready_to_arm_signals.insert({signal_name, signal});
    }
}
ArmedStateManager::~ArmedStateManager() {
}
eros_diagnostic::Diagnostic ArmedStateManager::update(double t_ros_time) {
    eros_diagnostic::Diagnostic diag = current_diagnostic;
    if (diag.message == eros_diagnostic::Message::INITIALIZING) {
        diag.message = eros_diagnostic::Message::NOERROR;
        diag.level = Level::Type::INFO;
        diag.description = "Updated";
    }
    current_diagnostic = diag;
    if (current_time < 0) {
        current_time = t_ros_time;
        return diag;
    }
    double prev_time = current_time;

    double delta_time = t_ros_time - prev_time;
    if (delta_time < 0) {
        diag.message = eros_diagnostic::Message::DIAGNOSTIC_FAILED;
        diag.level = Level::Type::ERROR;
        diag.description = "Went back in time!";
        current_diagnostic = diag;
        return diag;
    }
    current_time = t_ros_time;

    // Update all Timers for Ready to Arm Signals
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();
    bool all_ready_to_arm = true;
    bool any_ready_to_arm_timeout = false;
    for (; it != ready_to_arm_signals.end(); it++) {
        if (it->second.status == false) {
            all_ready_to_arm = false;
        }
        double delta_time = current_time - it->second.last_update_time;
        if (delta_time > ArmedStateManager::ARMED_SIGNAL_TIMEOUT_SEC) {
            any_ready_to_arm_timeout = true;
            it->second.signal_timeout = true;
        }
        else {
            it->second.signal_timeout = false;
        }
    }
    // State Machine for Armed State
    if ((all_ready_to_arm == false) || (any_ready_to_arm_timeout == true)) {
        current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
    }
    else if (current_armed_state.state == ArmDisarm::Type::DISARMED_CANNOTARM) {
        if ((all_ready_to_arm == true) && (any_ready_to_arm_timeout == false)) {
            current_armed_state.state = ArmDisarm::Type::DISARMED;
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::ARMING) {
        arming_timer += delta_time;
        if (arming_timer >= ArmedStateManager::ARMING_TIME_SEC) {
            current_armed_state.state = ArmDisarm::Type::ARMED;
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::DISARMING) {
        disarming_timer += delta_time;
        if (disarming_timer >= ArmedStateManager::DISARMING_TIME_SEC) {
            current_armed_state.state = ArmDisarm::Type::DISARMED;
        }
    }
    diag.message = eros_diagnostic::Message::NOERROR;
    diag.level = Level::Type::INFO;
    diag.description = "Updated";
    return diag;
}  // namespace eros
bool ArmedStateManager::new_command(eros::command cmd) {
    // Safe Armed State Change based on Commands
    if (current_armed_state.state == ArmDisarm::Type::DISARMED) {
        if (cmd.Command == (uint16_t)Command::Type::ARM) {
            current_armed_state.state = ArmDisarm::Type::ARMING;
            arming_timer = 0.0;
            return true;
        }
    }
    else if (current_armed_state.state == ArmDisarm::Type::ARMED) {
        if (cmd.Command == (uint16_t)Command::Type::DISARM) {
            current_armed_state.state = ArmDisarm::Type::DISARMING;
            disarming_timer = 0.0;
            return true;
        }
    }
    return false;
}

bool ArmedStateManager::new_ready_to_arm_msg(std::string signal, bool data) {
    std::map<std::string, ReadyToArmSignal>::iterator it;

    it = ready_to_arm_signals.find(signal);
    if (it != ready_to_arm_signals.end()) {
        it->second.status = data;
        it->second.last_update_time = current_time;
        return true;
    }
    return false;
}
bool ArmedStateManager::reset() {
    arming_timer = 0.0;
    disarming_timer = 0.0;
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();
    for (; it != ready_to_arm_signals.end(); it++) {
        it->second.status = false;
        it->second.last_update_time = -1.0;
        it->second.signal_timeout = true;
    }
    current_armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
    return true;
}
std::string ArmedStateManager::pretty() {
    std::string str = "-----Armed State Manager-----\n";
    str += "\tTime: " + std::to_string(current_time) + " (sec)\n";
    str += "\tReady to Arm Signals:\n";
    std::map<std::string, ReadyToArmSignal>::iterator it = ready_to_arm_signals.begin();

    for (; it != ready_to_arm_signals.end(); it++) {
        str += "\t\tSignal: " + it->second.signal_name +
               " Status: " + std::to_string(it->second.status) +
               " Last Update: " + std::to_string(it->second.last_update_time) + " (sec)" +
               " Timeout? " + std::to_string(it->second.signal_timeout) + "\n";
    }
    str += "\tArmed State: " + ArmDisarm::ArmDisarmString(current_armed_state.state) + "\n";
    str += "\tDiag: " + eros_diagnostic::DiagnosticUtility::pretty("", current_diagnostic, false) +
           "\n";
    return str;
}
}  // namespace eros