#include <eros/DiagnosticNode/DiagnosticNodeProcess.h>

DiagnosticNodeProcess::DiagnosticNodeProcess() {
}
DiagnosticNodeProcess::~DiagnosticNodeProcess() {
}
Diagnostic::DiagnosticDefinition DiagnosticNodeProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag;

    for (uint8_t i = 1; i < (uint8_t)(Diagnostic::DiagnosticType::END_OF_LIST); ++i) {
        DiagnosticTypeAggregate aggregate;
        Diagnostic::DiagnosticDefinition diag("NO DEVICE",
                                              "NO NODE",
                                              System::MainSystem::ROVER,
                                              System::SubSystem::ROBOT_CONTROLLER,
                                              System::Component::DIAGNOSTIC,
                                              (Diagnostic::DiagnosticType)i,
                                              Diagnostic::Message::NOERROR,
                                              Level::Type::INFO,
                                              "No Diagnostics Received Yet.");
        aggregate.worst_diag = diag;
        aggregate.worst_diag_key = build_key(diag);
        diagnostic_aggregator.insert(std::pair<Diagnostic::DiagnosticType, DiagnosticTypeAggregate>(
            (Diagnostic::DiagnosticType)i, aggregate));
    }
    return diag;
}
void DiagnosticNodeProcess::reset() {
}
Diagnostic::DiagnosticDefinition DiagnosticNodeProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    ready_to_arm.ready_to_arm = true;
    ready_to_arm.diag = convert(diag);
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> DiagnosticNodeProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> DiagnosticNodeProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
Diagnostic::DiagnosticDefinition DiagnosticNodeProcess::get_worst_diagnostic(
    Diagnostic::DiagnosticType type) {
    Diagnostic::DiagnosticDefinition empty_diag("NO DEVICE",
                                                "NO NODE",
                                                System::MainSystem::ROVER,
                                                System::SubSystem::ROBOT_CONTROLLER,
                                                System::Component::DIAGNOSTIC,
                                                Diagnostic::DiagnosticType::UNKNOWN,
                                                Diagnostic::Message::END_OF_LIST,
                                                Level::Type::FATAL,
                                                "Unknown Diagnostic Type: " + (uint8_t)type);
    auto aggregate_it = diagnostic_aggregator.find(type);
    if (aggregate_it == diagnostic_aggregator.end()) {
        return empty_diag;
    }
    else {
        if (aggregate_it->second.update_count == 0) {
            Diagnostic::DiagnosticDefinition diag("NO DEVICE",
                                                  "NO NODE",
                                                  System::MainSystem::ROVER,
                                                  System::SubSystem::ROBOT_CONTROLLER,
                                                  System::Component::DIAGNOSTIC,
                                                  type,
                                                  Diagnostic::Message::NODATA,
                                                  Level::Type::INFO,
                                                  "No Diagnostics Received Yet.");
            return diag;
        }
        else {
            return aggregate_it->second.worst_diag;
        }
    }
    return empty_diag;
}
Diagnostic::DiagnosticDefinition DiagnosticNodeProcess::update_topiclist(
    std::vector<std::string> &new_diagnostic_topics_to_subscribe) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    std::vector<std::string> new_diagnostic_topics;
    for (std::size_t i = 0; i < new_diagnostic_topics_to_subscribe.size(); ++i) {
        bool add_me = true;
        for (std::size_t j = 0; j < diagnostic_topics_already_monitoring.size(); ++j) {
            if (diagnostic_topics_already_monitoring.at(j) ==
                new_diagnostic_topics_to_subscribe.at(i)) {
                add_me = false;
            }
        }
        if (add_me == true) {
            new_diagnostic_topics.push_back(new_diagnostic_topics_to_subscribe.at(i));
            diagnostic_topics_already_monitoring.push_back(
                new_diagnostic_topics_to_subscribe.at(i));
        }
    }
    new_diagnostic_topics_to_subscribe = new_diagnostic_topics;
    return diag;
}
bool DiagnosticNodeProcess::new_external_diagnostic(Diagnostic::DiagnosticDefinition diag) {
    auto aggregate_it = diagnostic_aggregator.find(diag.type);
    if (aggregate_it == diagnostic_aggregator.end()) {
        return false;
    }
    std::string key = build_key(diag);
    if ((diag.level > aggregate_it->second.worst_diag.level)) {
        aggregate_it->second.worst_diag = diag;
        aggregate_it->second.worst_diag_key = key;
    }
    else if (key == aggregate_it->second.worst_diag_key) {
        aggregate_it->second.worst_diag = diag;
    }

    auto diag_it = aggregate_it->second.diag_list.find(key);
    if (diag_it == aggregate_it->second.diag_list.end()) {
        aggregate_it->second.diag_list.insert(
            std::pair<std::string, Diagnostic::DiagnosticDefinition>(key, diag));
    }
    else {
        diag_it->second = diag;
    }

    aggregate_it->second.update_count++;
    return true;
}
std::string DiagnosticNodeProcess::pretty() {
    std::string str = "---Diagnostic Aggregates---\n";
    uint8_t i = 1;
    for (auto aggregate_it : diagnostic_aggregator) {
        str += "[" + std::to_string(i) + "/" + std::to_string(diagnostic_aggregator.size()) + "] " +
               Diagnostic::DiagnosticTypeString(aggregate_it.first) +
               " Update Count: " + std::to_string(aggregate_it.second.update_count) +
               " Worst Level: " + Level::LevelString(aggregate_it.second.worst_diag.level) + "\n";
        if (aggregate_it.second.update_count > 0) {
            if (aggregate_it.second.worst_diag.level > Level::Type::NOTICE) {
                str += "\tWorst: " + Diagnostic::pretty("", aggregate_it.second.worst_diag, false) +
                       "\n";
            }
            uint16_t j = 1;
            for (auto diag_it : aggregate_it.second.diag_list) {
                str += "\t[" + std::to_string(j) + "/" +
                       std::to_string(aggregate_it.second.diag_list.size()) + "] " +
                       Diagnostic::pretty("", diag_it.second, false) + "\n";
                j++;
            }
        }

        i++;
    }
    return str;
}
std::string DiagnosticNodeProcess::build_key(Diagnostic::DiagnosticDefinition diag) {
    std::string key =
        diag.device_name + "_" + diag.node_name + "_" + std::to_string((uint8_t)diag.system) + "_" +
        std::to_string((uint8_t)diag.subsystem) + "_" + std::to_string((uint8_t)diag.component) +
        "_" + std::to_string((uint8_t)diag.type);
    return key;
}