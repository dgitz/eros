#include <eros/BaseNodeProcess.h>
Diagnostic::DiagnosticDefinition BaseNodeProcess::base_update(double t_dt, double t_system_time) {
    run_time += t_dt;
    system_time = t_system_time;
    Diagnostic::DiagnosticDefinition diag =
        diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                            Level::Type::DEBUG,
                                            Diagnostic::Message::NOERROR,
                                            "Base Process Updated.");
    return diag;
}
ros::Time BaseNodeProcess::convert_time(struct timeval t_) {
    ros::Time t;
    t.sec = t_.tv_sec;
    t.nsec = t_.tv_usec * 1000;
    return t;
}
ros::Time BaseNodeProcess::convert_time(double t_) {
    ros::Time t;
    t.sec = (int64_t)t_;
    double rem = t_ - (double)t.sec;
    t.nsec = (int64_t)(rem * 1000000.0);
    return t;
}
bool BaseNodeProcess::request_statechange(Node::State newstate) {
    Node::State current_state = node_state;
    bool state_changed = false;
    switch (current_state) {
        case Node::State::START:
            if (newstate == Node::State::INITIALIZING) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::INITIALIZING:
            if (newstate == Node::State::INITIALIZED) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::INITIALIZED:
            if (newstate == Node::State::RUNNING) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::RUNNING:
            if (newstate == Node::State::PAUSED) {
                state_changed = true;
            }
            else if (newstate == Node::State::RESET) {
                state_changed = true;
            }
            else if (newstate == Node::State::FINISHED) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::PAUSED:
            if (newstate == Node::State::RUNNING) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::RESET:
            if (newstate == Node::State::INITIALIZING) {
                state_changed = true;
            }
            else if (newstate == Node::State::RUNNING) {
                state_changed = true;
            }
            else {
                state_changed = false;
            }
            break;
        case Node::State::FINISHED: state_changed = false; break;
        default: state_changed = false; break;
    }
    if (state_changed == true) {
        node_state = newstate;
    }
    return state_changed;
}
std::vector<Diagnostic::DiagnosticDefinition> BaseNodeProcess::run_unittest() {
    std::vector<Diagnostic::DiagnosticDefinition> diaglist;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    if (unittest_running == false) {
        unittest_running = true;
        bool status = true;
        std::string data;
        std::string cmd =
            "cd ~/catkin_ws && "
            "bash devel/setup.bash && catkin_make run_tests_icarus_rover_v2_gtest_test_" +
            base_node_name +
            "_process >/dev/null 2>&1 && "
            "mv /home/robot/catkin_ws/build/test_results/icarus_rover_v2/gtest-test_" +
            base_node_name +
            "_process.xml "
            "/home/robot/catkin_ws/build/test_results/icarus_rover_v2/" +
            base_node_name + "/ >/dev/null 2>&1";
        // system(cmd.c_str());
        cmd =
            "cd ~/catkin_ws && bash devel/setup.bash && catkin_test_results "
            "build/test_results/icarus_rover_v2/" +
            base_node_name + "/";
        FILE *stream;

        const int max_buffer = 256;
        char buffer[max_buffer];
        cmd.append(" 2>&1");
        stream = popen(cmd.c_str(), "r");
        if (stream) {
            if (!feof(stream)) {
                if (fgets(buffer, max_buffer, stream) != NULL) {
                    data.append(buffer);
                }
                pclose(stream);
            }
        }
        std::vector<std::string> strs;
        std::size_t start = data.find(":");
        data.erase(0, start + 1);
        boost::split(strs, data, boost::is_any_of(",: "), boost::token_compress_on);
        if (strs.size() < 6) {
            char tempstr[1024];
            sprintf(tempstr, "Unable to process Unit Test Result: %s.", data.c_str());
            diag.description = std::string(tempstr);
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       std::string(tempstr));
            diaglist.push_back(diag);
            return diaglist;
        }
        int test_count = std::atoi(strs.at(1).c_str());
        int error_count = std::atoi(strs.at(3).c_str());
        int failure_count = std::atoi(strs.at(5).c_str());
        if (test_count == 0) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Test Count: 0");
            diaglist.push_back(diag);
            status = false;
        }
        if (error_count > 0) {
            char tempstr[512];
            sprintf(tempstr, "Error Count: %d.", error_count);
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       std::string(tempstr));
            diaglist.push_back(diag);
            status = false;
        }
        if (failure_count > 0) {
            char tempstr[512];
            sprintf(tempstr, "Failure Count: %d.", failure_count);
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       std::string(tempstr));
            diaglist.push_back(diag);
            status = false;
        }
        if (status == true) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       Level::Type::NOTICE,
                                                       Diagnostic::Message::NOERROR,
                                                       "Unit Test -> PASSED.");
            diaglist.push_back(diag);
        }
        else {
            Level::Type highest_error = Level::Type::INFO;
            for (std::size_t i = 0; i < diaglist.size(); i++) {
                if (diaglist.at(i).level > highest_error) {
                    highest_error = diaglist.at(i).level;
                }
            }
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                       highest_error,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Unit Test -> FAILED.");
            diaglist.push_back(diag);
        }
        unittest_running = false;
    }
    else {
        diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                   Level::Type::WARN,
                                                   Diagnostic::Message::DROPPING_PACKETS,
                                                   "Unit Test -> IS STILL IN PROGRESS.");
        diaglist.push_back(diag);
    }
    return diaglist;
}
eros::armed_state BaseNodeProcess::convert(ArmDisarm::State v) {
    eros::armed_state msg;
    msg.armed_state = (uint8_t)v.state;
    return msg;
}
eros::heartbeat BaseNodeProcess::convert_fromptr(const eros::heartbeat::ConstPtr &t_ptr) {
    eros::heartbeat msg;
    msg.stamp = t_ptr->stamp;
    msg.HostName = t_ptr->HostName;
    msg.BaseNodeName = t_ptr->BaseNodeName;
    msg.NodeName = t_ptr->NodeName;
    msg.NodeState = t_ptr->NodeState;
    return msg;
}
eros::command BaseNodeProcess::convert_fromptr(const eros::command::ConstPtr &t_ptr) {
    eros::command cmd;
    cmd.Command = t_ptr->Command;
    cmd.CommandText = t_ptr->CommandText;
    cmd.Description = t_ptr->Description;
    cmd.Option1 = t_ptr->Option1;
    cmd.Option2 = t_ptr->Option2;
    cmd.Option3 = t_ptr->Option3;
    return cmd;
}
eros::diagnostic BaseNodeProcess::convert_fromptr(const eros::diagnostic::ConstPtr &t_ptr) {
    eros::diagnostic diag;
    diag.Component = t_ptr->Component;
    diag.Description = t_ptr->Description;
    diag.DeviceName = t_ptr->DeviceName;
    diag.DiagnosticMessage = t_ptr->DiagnosticMessage;
    diag.DiagnosticType = t_ptr->DiagnosticType;
    diag.Level = t_ptr->Level;
    diag.NodeName = t_ptr->NodeName;
    diag.SubSystem = t_ptr->SubSystem;
    diag.System = t_ptr->System;
    return diag;
}
eros::resource BaseNodeProcess::convert_fromptr(const eros::resource::ConstPtr &t_ptr) {
    eros::resource msg;
    msg.stamp = t_ptr->stamp;
    msg.Name = t_ptr->Name;
    msg.PID = t_ptr->PID;
    msg.CPU_Perc = t_ptr->CPU_Perc;
    msg.RAM_Perc = t_ptr->RAM_Perc;
    msg.DISK_Perc = t_ptr->DISK_Perc;
    return msg;
}
eros::loadfactor BaseNodeProcess::convert_fromptr(const eros::loadfactor::ConstPtr &t_ptr) {
    eros::loadfactor msg;
    msg.stamp = t_ptr->stamp;
    msg.DeviceName = t_ptr->DeviceName;
    msg.loadfactor = t_ptr->loadfactor;
    return msg;
}
eros::command_state BaseNodeProcess::convert_fromptr(const eros::command_state::ConstPtr &t_ptr) {
    eros::command_state msg;
    msg.stamp = t_ptr->stamp;
    msg.Name = t_ptr->Name;
    msg.CurrentCommand = t_ptr->CurrentCommand;
    msg.State = t_ptr->State;
    msg.PercentComplete = t_ptr->PercentComplete;
    msg.diag = t_ptr->diag;
    return msg;
}
Diagnostic::DiagnosticDefinition BaseNodeProcess::convert(const eros::diagnostic diag) {
    Diagnostic::DiagnosticDefinition def;
    def.device_name = diag.DeviceName;
    def.node_name = diag.NodeName;
    def.system = (System::MainSystem)diag.System;
    def.subsystem = (System::SubSystem)diag.SubSystem;
    def.component = (System::Component)diag.Component;
    def.type = (Diagnostic::DiagnosticType)diag.DiagnosticType;
    def.message = (Diagnostic::Message)diag.DiagnosticMessage;
    def.level = (Level::Type)diag.Level;
    def.description = diag.Description;
    return def;
}
eros::diagnostic BaseNodeProcess::convert(const Diagnostic::DiagnosticDefinition def) {
    eros::diagnostic diag;
    diag.DeviceName = def.device_name;
    diag.NodeName = def.node_name;
    diag.System = (uint8_t)def.system;
    diag.SubSystem = (uint8_t)def.subsystem;
    diag.Component = (uint8_t)def.component;
    diag.DiagnosticType = (uint8_t)def.type;
    diag.DiagnosticMessage = (uint8_t)def.message;
    diag.Level = (uint8_t)def.level;
    diag.Description = def.description;
    return diag;
}
bool BaseNodeProcess::isEqual(double a, double b, double eps) {
    double dv = a - b;
    if (fabs(dv) < eps) {
        return true;
    }
    else {
        return false;
    }
}
std::string BaseNodeProcess::exec(const char *cmd, bool wait_for_result) {
    char buffer[512];
    std::string result = "";
    try {
        FILE *pipe = popen(cmd, "r");
        if (wait_for_result == false) {
            pclose(pipe);
            return "";
        }
        if (!pipe) {
            std::string tempstr = "popen() failed with command: " + std::string(cmd);
            logger->log_error(tempstr);
            pclose(pipe);
            return "";
        }
        try {
            while (!feof(pipe)) {
                if (fgets(buffer, 512, pipe) != NULL)
                    result += buffer;
            }
        }
        catch (const std::exception &e) {
            pclose(pipe);
            std::string tempstr = "popen() failed with command: " + std::string(cmd) +
                                  " and exception: " + std::string(e.what());
            logger->log_error(tempstr);
            return "";
        }
        pclose(pipe);
        boost::algorithm::trim(result);
        return result;
    }
    catch (const std::exception &e) {
        std::string tempstr = "popen() failed with command: " + std::string(cmd) +
                              " and exception: " + std::string(e.what());
        logger->log_error(tempstr);
        return "";
    }
}
void BaseNodeProcess::base_cleanup() {
    return;
}
json BaseNodeProcess::read_configuration(std::string device_name,
                                         bool include_self,
                                         std::string file_path) {
    json j_obj;
    json empty;
    std::ifstream fd(file_path);
    if (fd.is_open() == false) {
        logger->log_error("Unable to read file.");
        return empty;
    }
    json j;
    fd >> j_obj;
    if (include_self == true) {
        for (auto it = j_obj["DeviceList"].begin(); it != j_obj["DeviceList"].end(); ++it) {
            if (it.key() == device_name) {
                j[it.key()] = it.value();
            }
        }
    }
    for (auto it = j_obj["DeviceList"].begin(); it != j_obj["DeviceList"].end(); ++it) {
        if (it.value()["Parent"] == device_name) {
            j[it.key()] = it.value();
        }
    }
    return j;
}