#include "SnapshotProcess.h"

SnapshotProcess::SnapshotProcess()
    : mode(Mode::UNKNOWN),
      architecture(Architecture::Type::UNKNOWN),
      devicesnapshot_state(SnapshotState::NOTRUNNING),
      systemsnapshot_state(SnapshotState::NOTRUNNING) {
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
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::new_commandstatemsg(
    eros::command_state t_msg) {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    if (mode == Mode::MASTER) {
        if (t_msg.Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
            if (t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE) {
                for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                    if (snapshot_config.snapshot_devices.at(i).name == t_msg.NodeName) {
                        if (t_msg.State == 1) {
                            snapshot_config.snapshot_devices.at(i).device_snapshot_generated = true;
                        }
                    }
                }
            }
        }
    }
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::new_commandmsg(eros::command t_msg) {
    Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    if (t_msg.Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
        if (((mode == Mode::MASTER) &&
             (t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER)) ||
            ((mode == Mode::SLAVE) &&
             (t_msg.Option1 == (uint16_t)Command::GenerateSnapshot_Option1::RUN_SLAVE))) {
            if ((systemsnapshot_state != SnapshotState::NOTRUNNING) ||
                (devicesnapshot_state != SnapshotState::NOTRUNNING)) {
                diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                         Level::Type::WARN,
                                         Diagnostic::Message::DROPPING_PACKETS,
                                         "Snapshot is still being generated.");
                diag_list.push_back(diag);
                return diag_list;
            }
            else {
                if (mode == Mode::MASTER) {
                    systemsnapshot_state = SnapshotState::STARTED;
                }
                devicesnapshot_state = SnapshotState::STARTED;
                diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                         Level::Type::INFO,
                                         Diagnostic::Message::NOERROR,
                                         "Snapshot Started.");
            }
        }
        else {
            logger->log_debug("Command Option1: " + std::to_string(t_msg.Option1) +
                              " Not meant for me: " + std::to_string((uint8_t)mode));
        }
    }
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    return diag_list;
}
std::string SnapshotProcess::pretty() {
    std::string str = "--- Snapshot Config ---\n";
    str += " Mode: " + ModeString(mode) + "\n";
    if (mode == Mode::MASTER) {
        str += " System Snapshot State: " + SnapshotStateString(systemsnapshot_state) + "\n";
        if (snapshot_config.snapshot_devices.size() == 0) {
            str += " NO Snapshot Devices Defined.\n";
        }
        else {
            for (std::size_t i = 0; i < snapshot_config.snapshot_devices.size(); ++i) {
                str += " Device: " + snapshot_config.snapshot_devices.at(i).name + "\n";
            }
        }
    }
    str += " Device Snapshot State: " + SnapshotStateString(devicesnapshot_state) + "\n";
    str += " Stage Directory: " + snapshot_config.stage_directory + "\n";
    str += " Device Snapshot Path: " + snapshot_config.device_snapshot_path + "\n";
    if (snapshot_config.commands.size() == 0) {
        str += " No Commands Defined.\n";
    }
    else {
        str += " Commands:\n";
        for (std::size_t i = 0; i < snapshot_config.commands.size(); ++i) {
            str += "\t[" + std::to_string(i) + "/" +
                   std::to_string(snapshot_config.commands.size()) +
                   "] Cmd: " + snapshot_config.commands.at(i).command + "\n";
        }
    }
    return str;
}
std::vector<Diagnostic::DiagnosticDefinition> SnapshotProcess::createnew_snapshot() {
    // logger->log_notice("starting");
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();

    devicesnapshot_state = SnapshotState::RUNNING;
    // Clean up Stage Directory
    {
        try {
            std::string rm_cmd = "rm -r -f " + snapshot_config.stage_directory;
            exec(rm_cmd.c_str(), true);
            std::string mkdir_cmd = "mkdir -p " + snapshot_config.stage_directory;
            exec(mkdir_cmd.c_str(), true);
            mkdir_cmd = "mkdir -p " + snapshot_config.device_snapshot_path;
            exec(mkdir_cmd.c_str(), true);
        }
        catch (const std::exception &e) {
            diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::ERROR,
                                     Diagnostic::Message::DROPPING_PACKETS,
                                     "Command Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    // Run Snapshot "Commands"
    for (std::size_t i = 0; i < snapshot_config.commands.size(); ++i) {
        char tempstr[1024];
        sprintf(tempstr,
                "%s > %s/%s",
                snapshot_config.commands.at(i).command.c_str(),
                snapshot_config.stage_directory.c_str(),
                snapshot_config.commands.at(i).output_file.c_str());
        try {
            exec(tempstr, true);
        }
        catch (const std::exception &e) {
            diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::ERROR,
                                     Diagnostic::Message::DROPPING_PACKETS,
                                     "Command Exec failed with error: " + std::string(e.what()));
            diag_list.push_back(diag);
            return diag_list;
        }
    }
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "Snapshot_%Y_%m_%d_%H_%M_%S", timeinfo);
    std::string str(buffer);
    std::string snapshot_name = get_hostname() + "_" + str;
    std::string active_snapshot_completepath = "";
    // Zip up Device Snapshot
    if (1) {
        char tempstr[1024];
        sprintf(tempstr,
                "cd %s && zip -r %s/%s.zip .",
                snapshot_config.stage_directory.c_str(),
                snapshot_config.device_snapshot_path.c_str(),
                snapshot_name.c_str());
        active_snapshot_completepath =
            snapshot_config.device_snapshot_path + snapshot_name + ".zip";
        logger->log_notice("Running: " + std::string(tempstr));
        exec(tempstr, true);
    }
    // Make sure it's actually there
    {
        int file_found =
            count_files_indirectory(snapshot_config.device_snapshot_path, snapshot_name + ".zip");
        if (file_found != 1) {
            diag = update_diagnostic(
                Diagnostic::DiagnosticType::DATA_STORAGE,
                Level::Type::WARN,
                Diagnostic::Message::DROPPING_PACKETS,
                "Device Snapshot not created at: " + active_snapshot_completepath);
            diag_list.push_back(diag);
            return diag_list;
        }
        else {
            devicesnapshot_state =
                SnapshotState::NOTRUNNING;  // Mark NOT RUNNING For now, may need to change when
                                            // System Snapshot gets implemented.
            diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::INFO,
                                     Diagnostic::Message::NOERROR,
                                     "Device Snapshot Created at: " + active_snapshot_completepath);
            diag_list.push_back(diag);
        }
    }
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
    std::vector<std::string> missing_required_keys;
    std::vector<std::string> missing_optional_keys;
    TiXmlElement *l_pRootElement = doc.RootElement();
    bool found_architecture = false;
    if (nullptr != l_pRootElement) {
        TiXmlElement *l_pSnapshotConfig = l_pRootElement->FirstChildElement("SnapshotConfig");
        if (nullptr != l_pSnapshotConfig) {
            if (mode == Mode::MASTER) {
                TiXmlElement *l_pSnapshotDevices =
                    l_pSnapshotConfig->FirstChildElement("SnapshotDevices");
                if (nullptr != l_pSnapshotDevices) {
                    TiXmlElement *l_pSnapshotDevice =
                        l_pSnapshotDevices->FirstChildElement("Device");
                    while (l_pSnapshotDevice) {
                        std::string device_name = l_pSnapshotDevice->GetText();
                        if (device_name != get_hostname()) {
                            SlaveDevice newSlave(device_name);
                            snapshot_config.snapshot_devices.push_back(newSlave);
                        }
                        l_pSnapshotDevice = l_pSnapshotDevice->NextSiblingElement("Device");
                    }
                    if (snapshot_config.snapshot_devices.size() == 0) {
                        missing_required_keys.push_back("SnapshotDevices/Device");
                    }
                }
                else {
                    missing_required_keys.push_back("SnapshotDevices");
                }
            }
            TiXmlElement *l_pStageDirectory =
                l_pSnapshotConfig->FirstChildElement("StageDirectory");
            if (nullptr != l_pStageDirectory) {
                snapshot_config.stage_directory =
                    std::string(l_pStageDirectory->GetText()) + "/DeviceSnapshot";
            }
            else {
                missing_required_keys.push_back("StageDirectory");
            }

            TiXmlElement *l_pArchitecture = l_pSnapshotConfig->FirstChildElement("Architecture");
            if (nullptr != l_pArchitecture) {
                while (l_pArchitecture) {
                    if (l_pArchitecture->Attribute("type") ==
                        Architecture::ArchitectureString(architecture)) {
                        found_architecture = true;
                        TiXmlElement *l_pDeviceSnapshotPath =
                            l_pArchitecture->FirstChildElement("DeviceSnapshotPath");
                        if (nullptr != l_pDeviceSnapshotPath) {
                            snapshot_config.device_snapshot_path = l_pDeviceSnapshotPath->GetText();
                        }
                        else {
                            missing_required_keys.push_back("DeviceSnapshotPath");
                        }
                        TiXmlElement *l_pCommand = l_pArchitecture->FirstChildElement("Command");
                        if (nullptr != l_pCommand) {
                            while (l_pCommand) {
                                ExecCommand cmd;
                                cmd.command = l_pCommand->GetText();
                                cmd.output_file = l_pCommand->Attribute("file");
                                snapshot_config.commands.push_back(cmd);
                                l_pCommand = l_pCommand->NextSiblingElement("Command");
                            }
                        }
                    }
                    l_pArchitecture = l_pArchitecture->NextSiblingElement("Architecture");
                }
            }
        }
    }
    if (found_architecture == false) {
        missing_required_keys.push_back("Architecture");
    }
    if (missing_required_keys.size() == 0) {
        if (missing_optional_keys.size() == 0) {
            diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::NOTICE,
                                     Diagnostic::Message::NOERROR,
                                     "Config File Loaded.");
        }
        else {
            std::string str = "";
            for (std::size_t i = 0; i < missing_optional_keys.size(); ++i) {
                if (i == 0) {
                    str = missing_optional_keys.at(i);
                }
                else {
                    str += "," + missing_optional_keys.at(i);
                }
            }
            diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                     Level::Type::WARN,
                                     Diagnostic::Message::NOERROR,
                                     "Missing Optional Keys: " + str);
        }
    }
    else {
        std::string str = "";
        for (std::size_t i = 0; i < missing_required_keys.size(); ++i) {
            if (i == 0) {
                str = missing_required_keys.at(i);
            }
            else {
                str += "," + missing_required_keys.at(i);
            }
        }
        diag = update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                 Level::Type::ERROR,
                                 Diagnostic::Message::INITIALIZING_ERROR,
                                 "Missing Required Keys: " + str);
    }
    return diag;
}
int SnapshotProcess::count_files_indirectory(std::string directory, std::string filter) {
    try {
        char tempstr[1024];
        sprintf(tempstr, "ls %s%s 2>/dev/null | wc -l", directory.c_str(), filter.c_str());
        std::string return_v = exec(tempstr, true);
        boost::trim_right(return_v);
        return std::atoi(return_v.c_str());
    }
    catch (const std::exception &e) {
        return 0;
    }
}