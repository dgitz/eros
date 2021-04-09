#include <eros/SafetyNode/SafetyNodeProcess.h>
ArmDisarmMonitor::ArmDisarmMonitor(std::string _name, Type _type) : name(_name), type(_type) {
    ready_to_arm.ready_to_arm = false;
    ready_to_arm.diag.Description = "Nothing Received";
    update_count = 0;
    last_delta_update_time = 0.0;
}
ArmDisarmMonitor::~ArmDisarmMonitor() {
}
SafetyNodeProcess::SafetyNodeProcess() {
    armed_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;  // No monitors defined yet.
}
SafetyNodeProcess::~SafetyNodeProcess() {
}
Diagnostic::DiagnosticDefinition SafetyNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;
    diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                             Level::Type::INFO,
                             Diagnostic::Message::NODATA,
                             "No Remote Control Command Yet.");
    return diag;
}
void SafetyNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition SafetyNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    ArmDisarm::State prev_arm_state = armed_state;
    ArmDisarm::State temp_arm_state = armed_state;
    std::vector<std::string> temp_cannotarm_reasons;
    bool monitors_ok = true;
    if (ready_to_arm_monitors.size() == 0) {
        monitors_ok = false;
    }
    else {
        std::map<std::string, ArmDisarmMonitor>::iterator it;
        for (it = ready_to_arm_monitors.begin(); it != ready_to_arm_monitors.end(); it++) {
            it->second.last_delta_update_time += t_dt;
            if (it->second.update_count == 0) {
                std::string str = it->second.name + " Never Received Data";
                temp_cannotarm_reasons.push_back(str);
                monitors_ok = false;
            }
            else if (it->second.ready_to_arm.ready_to_arm == false) {
                std::string str = it->second.name + " " + it->second.ready_to_arm.diag.Description;
                temp_cannotarm_reasons.push_back(str);
                monitors_ok = false;
            }
            else if (it->second.last_delta_update_time > ArmDisarmMonitor::READYTOARM_TIMEOUT) {
                temp_cannotarm_reasons.push_back("Have not heard from: " + it->first + " In " +
                                                 std::to_string(it->second.last_delta_update_time) +
                                                 " Seconds.");
                monitors_ok = false;
            }
        }
    }

    if (monitors_ok == false) {
        temp_arm_state.state = ArmDisarm::Type::DISARMED_CANNOTARM;
        cannotarm_reasons = temp_cannotarm_reasons;
    }
    else {
        if (temp_arm_state.state == ArmDisarm::Type::DISARMED_CANNOTARM) {
            temp_arm_state.state = ArmDisarm::Type::DISARMED;
            cannotarm_reasons.clear();
        }
    }
    if (temp_arm_state.state == ArmDisarm::Type::ARMING) {
        temp_arm_state.state = ArmDisarm::Type::ARMED;
    }
    else if (temp_arm_state.state == ArmDisarm::Type::DISARMING) {
        temp_arm_state.state = ArmDisarm::Type::DISARMED;
    }
    armed_state = temp_arm_state;
    if (armed_state.state != prev_arm_state.state) {
        logger->log_notice(
            "Arm State Changed From: " + ArmDisarm::ArmDisarmString(prev_arm_state.state) +
            " To: " + ArmDisarm::ArmDisarmString(armed_state.state));
    }
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SafetyNodeProcess::new_commandmsg(eros::command msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    if (msg.Command == (uint16_t)Command::Type::ARM) {
        if (armed_state.state == ArmDisarm::Type::DISARMED) {
            armed_state.state = ArmDisarm::Type::ARMING;
            diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                     Level::Type::NOTICE,
                                     Diagnostic::Message::NOERROR,
                                     "System Arming.");
            diag_list.push_back(diag);
        }
        else if (armed_state.state == ArmDisarm::Type::DISARMED_CANNOTARM) {
            diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                     Level::Type::WARN,
                                     Diagnostic::Message::DIAGNOSTIC_FAILED,
                                     "System Disarmed and Cannot Arm.");
            diag_list.push_back(diag);
        }
    }
    else if (msg.Command == (uint16_t)Command::Type::DISARM) {
        if (armed_state.state == ArmDisarm::Type::ARMED) {
            armed_state.state = ArmDisarm::Type::DISARMING;
            diag = update_diagnostic(Diagnostic::DiagnosticType::REMOTE_CONTROL,
                                     Level::Type::NOTICE,
                                     Diagnostic::Message::NOERROR,
                                     "System Disarming.");
            diag_list.push_back(diag);
        }
    }
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SafetyNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
bool SafetyNodeProcess::new_message_readytoarm(std::string name, eros::ready_to_arm ready_to_arm) {
    std::map<std::string, ArmDisarmMonitor>::iterator it = ready_to_arm_monitors.find(name);
    if (it != ready_to_arm_monitors.end()) {
        if (it->second.type == ArmDisarmMonitor::Type::DEFAULT) {
            it->second.ready_to_arm = ready_to_arm;
            it->second.update_count++;
            it->second.last_delta_update_time = 0.0;
            diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                Level::Type::DEBUG,
                                                Diagnostic::Message::NOERROR,
                                                "Received Default ReadyToArm Msg");
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}
bool SafetyNodeProcess::new_message_readytoarm(std::string name, bool v) {
    std::map<std::string, ArmDisarmMonitor>::iterator it = ready_to_arm_monitors.find(name);
    if (it != ready_to_arm_monitors.end()) {
        if (it->second.type == ArmDisarmMonitor::Type::SIMPLE) {
            it->second.ready_to_arm.ready_to_arm = v;
            if (v == true) {
                it->second.ready_to_arm.diag.Description = " Ready To Arm";
            }
            else {
                it->second.ready_to_arm.diag.Description = " Not Ready To Arm";
            }
            it->second.update_count++;
            it->second.last_delta_update_time = 0.0;
            diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                Level::Type::DEBUG,
                                                Diagnostic::Message::NOERROR,
                                                "UpReceived Simple ReadyToArm Msgdated");
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool SafetyNodeProcess::initialize_readytoarm_monitors(std::vector<std::string> topics,
                                                       std::vector<ArmDisarmMonitor::Type> types) {
    if (topics.size() != types.size()) {
        logger->log_error("Topic/Type Size Mismatch.");
        return false;
    }
    if (topics.size() == 0) {
        logger->log_warn("No Arm Monitors Defined.");
        return true;
    }
    int counter = 0;
    for (auto type : types) {
        if ((type == ArmDisarmMonitor::Type::UNKNOWN) ||
            (type == ArmDisarmMonitor::Type::END_OF_LIST)) {
            logger->log_error("Arm/Disarm Type at Index: " + std::to_string(counter) +
                              " Is not fully Defined.");
            return false;
        }
        counter++;
    }
    counter = 0;
    for (auto topic : topics) {
        ArmDisarmMonitor newMonitor = ArmDisarmMonitor(topic, types.at(counter));

        ready_to_arm_monitors.insert(std::pair<std::string, ArmDisarmMonitor>(topic, newMonitor));
        counter++;
    }
    bool init_ok = true;
    if (ready_to_arm_monitors.size() != topics.size()) {
        logger->log_error("Monitor/Topic Size Mismatch.");
        init_ok = false;
    }

    return init_ok;
}
bool SafetyNodeProcess::init_ros(boost::shared_ptr<ros::NodeHandle> _n) {
    nodeHandle = _n;
    for (auto monitor : ready_to_arm_monitors) {
        if (monitor.second.type == ArmDisarmMonitor::Type::DEFAULT) {
            ros::Subscriber sub = nodeHandle->subscribe<eros::ready_to_arm>(
                monitor.second.name,
                10,
                boost::bind(
                    &SafetyNodeProcess::ReadyToArmDefaultCallback, this, _1, monitor.second.name));
            arm_monitor_subs.push_back(sub);
        }
        else if (monitor.second.type == ArmDisarmMonitor::Type::SIMPLE) {
            ros::Subscriber sub = nodeHandle->subscribe<std_msgs::Bool>(
                monitor.second.name,
                10,
                boost::bind(
                    &SafetyNodeProcess::ReadyToArmSimpleCallback, this, _1, monitor.second.name));
            arm_monitor_subs.push_back(sub);
        }
        else {
            return false;
        }
    }
    return true;
}
void SafetyNodeProcess::ReadyToArmDefaultCallback(const eros::ready_to_arm::ConstPtr &msg,
                                                  const std::string &topic_name) {
    if (new_message_readytoarm(topic_name, BaseNodeProcess::convert_fromptr(msg)) == false) {
        logger->log_warn("Unable to process Topic: " + topic_name);
    }
}
void SafetyNodeProcess::ReadyToArmSimpleCallback(const std_msgs::Bool::ConstPtr &msg,
                                                 const std::string &topic_name) {
    bool v = msg->data;
    if (new_message_readytoarm(topic_name, v) == false) {
        logger->log_warn("Unable to process Topic: " + topic_name);
    }
}