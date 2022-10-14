#include <eros/BaseNodeProcess.h>
using namespace eros;
Diagnostic::DiagnosticDefinition BaseNodeProcess::base_update(double t_dt, double t_system_time) {
    run_time += t_dt;
    system_time = t_system_time;
    Diagnostic::DiagnosticDefinition diag =
        diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                            Level::Type::DEBUG,
                                            Diagnostic::Message::NOERROR,
                                            "Base Process Updated.");
    if (node_state == Node::State::START) {
        request_statechange(Node::State::INITIALIZING);
    }
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
        // No practical way to enter, keeping just in case.
        // LCOV_EXCL_START
        default:
            state_changed = false;
            break;
            // LCOV_EXCL_STOP
    }
    if (state_changed == true) {
        node_state = newstate;
    }
    else {
        logger->log_info("State Change from: " + Node::NodeStateString(current_state) +
                         " To State: " + Node::NodeStateString(newstate) + " Rejected.");
    }
    return state_changed;
}
eros::armed_state BaseNodeProcess::convert(ArmDisarm::State v) {
    eros::armed_state msg;
    msg.armed_state = (uint8_t)v.state;
    return msg;
}
ArmDisarm::State BaseNodeProcess::convert(eros::armed_state v) {
    ArmDisarm::State data;
    data.state = (ArmDisarm::Type)v.armed_state;
    return data;
}
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::ready_to_arm BaseNodeProcess::convert_fromptr(const eros::ready_to_arm::ConstPtr &t_ptr) {
    eros::ready_to_arm msg;
    msg.ready_to_arm = t_ptr->ready_to_arm;
    msg.diag = t_ptr->diag;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::heartbeat BaseNodeProcess::convert_fromptr(const eros::heartbeat::ConstPtr &t_ptr) {
    eros::heartbeat msg;
    msg.stamp = t_ptr->stamp;
    msg.HostName = t_ptr->HostName;
    msg.BaseNodeName = t_ptr->BaseNodeName;
    msg.NodeName = t_ptr->NodeName;
    msg.NodeState = t_ptr->NodeState;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
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
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
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
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
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
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
eros::loadfactor BaseNodeProcess::convert_fromptr(const eros::loadfactor::ConstPtr &t_ptr) {
    eros::loadfactor msg;
    msg.stamp = t_ptr->stamp;
    msg.DeviceName = t_ptr->DeviceName;
    msg.loadfactor = t_ptr->loadfactor;
    return msg;
}
// LCOV_EXCL_STOP
// No Obvious way to Unit test
// LCOV_EXCL_START
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
// LCOV_EXCL_STOP
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

void BaseNodeProcess::base_cleanup() {
    return;
}
json BaseNodeProcess::read_configuration(std::string device_name,
                                         bool include_self,
                                         std::string file_path) {
    json j_obj;
    json empty;
    if (file_path.size() == 0) {
        logger->log_error("No file path defined.");
        return empty;
    }
    file_path = sanitize_path(file_path);
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
std::string BaseNodeProcess::sanitize_path(std::string path) {
    if (path.at(0) == '~') {
        path.erase(path.begin() + 0);
        path = std::string(getenv("HOME")) + path;
    }
    return path;
}

FileHelper::FileInfo BaseNodeProcess::read_file(std::string file_path) {
    FileHelper::FileInfo fileInfo;
    file_path = sanitize_path(file_path);
    file_path = boost::filesystem::absolute(file_path).string();
    fileInfo.full_path = file_path;
    std::string extension = boost::filesystem::extension(file_path);
    if (extension == ".zip") {
        fileInfo.fileType = FileHelper::FileType::ZIP;
    }
    else {
        fileInfo.fileType = FileHelper::FileType::UNKNOWN;
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    auto p = boost::filesystem::path(file_path);
    fileInfo.folder = p.parent_path().string() + "/";
    fileInfo.file_name = p.filename().string();
    std::ifstream fl(file_path);
    if (fl.is_open() == false) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    fl.seekg(0, std::ios::end);
    std::size_t len = fl.tellg();
    char *ret = new char[len];
    fl.seekg(0, std::ios::beg);
    fl.read(ret, len);
    fl.close();
    fileInfo.data = ret;
    fileInfo.byte_size = (uint64_t)len;
    if (fileInfo.byte_size > 0) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_OK;
    }
    else {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
    }
    return fileInfo;
}
FileHelper::FileInfo BaseNodeProcess::write_file(std::string full_path,
                                                 char *bytes,
                                                 uint64_t byte_count) {
    FileHelper::FileInfo fileInfo;
    full_path = sanitize_path(full_path);
    full_path = boost::filesystem::absolute(full_path).string();
    fileInfo.full_path = full_path;
    std::string extension = boost::filesystem::extension(full_path);
    if (extension == ".zip") {
        fileInfo.fileType = FileHelper::FileType::ZIP;
    }
    else {
        fileInfo.fileType = FileHelper::FileType::UNKNOWN;
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    auto p = boost::filesystem::path(full_path);
    fileInfo.folder = p.parent_path().string() + "/";
    fileInfo.file_name = p.filename().string();
    std::ofstream file(full_path.c_str(), std::ios::binary);
    if (file.is_open() == false) {
        fileInfo.fileStatus = FileHelper::FileStatus::FILE_ERROR;
        return fileInfo;
    }
    file.write(bytes, byte_count);
    file.close();
    fileInfo.data = bytes;
    fileInfo.byte_size = byte_count;
    fileInfo.fileStatus = FileHelper::FileStatus::FILE_OK;
    return fileInfo;
}
std::vector<std::string> BaseNodeProcess::get_files_indir(std::string dir) {
    std::vector<std::string> files;
    std::string ls_cmd = "ls " + dir;
    ExecResult execResult = exec(ls_cmd.c_str(), true);
    std::string res = execResult.Result;
    boost::split(files, res, boost::is_any_of("\n"), boost::token_compress_on);
    for (std::vector<std::string>::iterator it = files.begin(); it != files.end();) {
        if (it->size() == 0) {
            it = files.erase(it);
        }
        else {
            ++it;
        }
    }
    return files;
}