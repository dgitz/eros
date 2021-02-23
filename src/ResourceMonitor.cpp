#include <eros/ResourceMonitor.h>
ResourceMonitor::ResourceMonitor() {
}
ResourceMonitor::ResourceMonitor(Mode _mode,
                                 Diagnostic::DiagnosticDefinition _diag,
                                 Logger *_logger)
    : mode(_mode),
      architecture(Architecture::Type::UNKNOWN),
      diagnostic(_diag),
      logger(_logger),
      initialized(false),
      run_time(0.0),
      processor_count(0) {
    resourceInfo.cpu_perc = -1.0;
    resourceInfo.disk_perc = -1.0;
    resourceInfo.ram_perc = -1.0;
    diagnostic.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diagnostic.update_count = 0;
    load_factor.push_back(0.0);
    load_factor.push_back(0.0);
    load_factor.push_back(0.0);
}
ResourceMonitor::~ResourceMonitor() {
}
Diagnostic::DiagnosticDefinition ResourceMonitor::init() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    architecture = read_device_architecture();
    if (architecture == Architecture::Type::UNKNOWN) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
        initialized = false;
    }
    if ((architecture != Architecture::Type::X86_64) ||
        (architecture != Architecture::Type::ARMV7L) ||
        (architecture != Architecture::Type::AARCH64)) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description =
            "Architecture: " + Architecture::ArchitectureString(architecture) + " Not Supported.";
        diag.update_count++;
        initialized = false;
    }
    resourceInfo.pid = ::getpid();
    try {
        processor_count = std::atoi(exec("nproc", true).c_str());
    }
    catch (const std::exception &e) {
        std::string tempstr = "Unable to determine number of processors: " + std::string(e.what());

        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = tempstr;
        diag.update_count++;
        logger->log_error(tempstr);
        initialized = false;
    }
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
        if (diag.level > Level::Type::WARN) {
            return diag;
        }
        diag = read_device_loadfactor();
    }
    if (diag.level <= Level::Type::NOTICE) {
        initialized = true;
        logger->log_diagnostic(diag);
    }
    else {
        initialized = false;
    }
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::update(double t_dt) {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    run_time += t_dt;
    if (architecture == Architecture::Type::UNKNOWN) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
    }
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
        if (diag.level > Level::Type::WARN) {
            return diag;
        }
        diag = read_device_loadfactor();
    }
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::read_process_resource_usage() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string top_query =
        "top -b -n 2 -d 0.2 -p " + std::to_string(resourceInfo.pid) + " | tail -1";

    std::string res = exec(top_query.c_str(), true);
    std::vector<std::string> strs;
    boost::algorithm::split(strs, res, boost::is_any_of("\t "), boost::token_compress_on);
    if (strs.at(0) == "") {
        strs.erase(strs.begin());
    }
    if (architecture == Architecture::Type::X86_64) {
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process (size != 12): " + res;
            diag.update_count++;
            return diag;
        }
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    else if (architecture == Architecture::Type::AARCH64) {
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process: (size != 12)" + res;
            diag.update_count++;
            return diag;
        }
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    else if (architecture == Architecture::Type::ARMV7L) {
        if (strs.size() != 12) {
            for (std::size_t i = 0; i < strs.size(); ++i) {
                printf("[%d/%d] %s\n", (int)i, (int)strs.size(), strs.at(i).c_str());
            }
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description = "Improper string to process: (size != 13)" + res;
            diag.update_count++;
            return diag;
        }
        try {
            resourceInfo.cpu_perc = std::atof(strs.at(8).c_str());
            resourceInfo.ram_perc = std::atof(strs.at(9).c_str());
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    else {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description =
            "Architecture: " + Architecture::ArchitectureString(architecture) + " Not Supported.";
        diag.update_count++;
        return diag;
    }
    diag.message = Diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::read_device_resource_availability() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    {  // Read Free CPU
        std::string res;
        if ((architecture == Architecture::Type::X86_64) ||
            (architecture == Architecture::Type::AARCH64) ||
            (architecture == Architecture::Type::ARMV7L)) {
            try {
                std::string top_query = "top -bn1 | grep '%Cpu(s)'";
                std::string res = exec(top_query.c_str(), true);
                std::vector<std::string> strs;
                boost::algorithm::split(
                    strs, res, boost::is_any_of("\t "), boost::token_compress_on);
                bool found_me = false;
                for (std::size_t i = 0; i < strs.size(); ++i) {
                    if (strs.at(i).find("id") != std::string::npos) {
                        if (i > 0) {
                            found_me = true;
                            resourceInfo.cpu_perc = std::atof(strs.at(i - 1).c_str());
                        }
                    }
                }
                if (found_me == false) {
                    diag.level = Level::Type::ERROR;
                    diag.message = Diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
            }
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = Diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
        }
        diag.level = Level::Type::INFO;
        diag.message = Diagnostic::Message::NOERROR;
        diag.description = "Updated.";
        diag.update_count++;
    }
    {  // Read Free RAM
        std::string res;
        if ((architecture == Architecture::Type::X86_64) ||
            (architecture == Architecture::Type::AARCH64) ||
            (architecture == Architecture::Type::ARMV7L)) {
            try {
                std::string top_query = "top -bn1 | grep 'Mem'";
                std::string res = exec(top_query.c_str(), true);
                std::vector<std::string> strs;
                boost::algorithm::split(
                    strs, res, boost::is_any_of("\t "), boost::token_compress_on);
                int found_count = 0;
                int64_t total_mem = 0;
                int64_t used_mem = 0;
                for (std::size_t i = 0; i < strs.size(); ++i) {
                    if (strs.at(i).find("used") != std::string::npos) {
                        if (i > 0) {
                            found_count++;
                            used_mem = std::atoi(strs.at(i - 1).c_str());
                        }
                    }
                    if (strs.at(i).find("total") != std::string::npos) {
                        if (i > 0) {
                            found_count++;
                            total_mem = std::atoi(strs.at(i - 1).c_str());
                        }
                    }
                    if (found_count == 2) {
                        break;
                    }
                }
                if (found_count != 2) {
                    diag.level = Level::Type::ERROR;
                    diag.message = Diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
                else {
                    resourceInfo.ram_perc =
                        100.0 - (100.0 * (double)(used_mem) / (double)(total_mem));
                }
            }
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = Diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
        }
    }
    {  // Read Free Disk Space
        std::string res;
        if ((architecture == Architecture::Type::X86_64) ||
            (architecture == Architecture::Type::AARCH64) ||
            (architecture == Architecture::Type::ARMV7L)) {
            try {
                std::string df_query = "df -h";
                std::string res = exec(df_query.c_str(), true);
                std::vector<std::string> lines;
                boost::split(lines, res, boost::is_any_of("\n"), boost::token_compress_on);
                bool found_me = false;
                for (std::size_t i = 0; i < lines.size(); ++i) {
                    std::vector<std::string> fields;
                    boost::split(
                        fields, lines.at(i), boost::is_any_of(" "), boost::token_compress_on);
                    std::string mount_point = fields.at(fields.size() - 1);
                    if (mount_point == "/") {
                        try {
                            found_me = true;
                            std::string tempstr1 = fields.at(4);

                            tempstr1 =
                                tempstr1.substr(0, tempstr1.size() - 1);  // Remove trailing % sign
                            resourceInfo.disk_perc = 100.0 - std::atof(tempstr1.c_str());
                        }
                        catch (const std::exception &e) {
                            diag.level = Level::Type::ERROR;
                            diag.message = Diagnostic::Message::DROPPING_PACKETS;
                            diag.description = "Unable to process string: " + res +
                                               " with result: " + std::string(e.what());
                            diag.update_count++;
                            return diag;
                        }
                    }
                }
                if (found_me == false) {
                    diag.level = Level::Type::ERROR;
                    diag.message = Diagnostic::Message::DROPPING_PACKETS;
                    diag.description = "Unable to process string: " + res;
                    diag.update_count++;
                    return diag;
                }
            }
            catch (const std::exception &e) {
                diag.level = Level::Type::ERROR;
                diag.message = Diagnostic::Message::DROPPING_PACKETS;
                diag.description =
                    "Unable to process string: " + res + " with result: " + std::string(e.what());
                diag.update_count++;
                return diag;
            }
        }
    }
    diag.level = Level::Type::INFO;
    diag.message = Diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::read_device_loadfactor() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string res;
    if ((architecture == Architecture::Type::X86_64) ||
        (architecture == Architecture::Type::AARCH64) ||
        (architecture == Architecture::Type::ARMV7L)) {
        try {
            std::string top_query = "top -bn1 | grep 'load average:'";
            std::string res = exec(top_query.c_str(), true);
            std::vector<std::string> strs;
            boost::algorithm::split(strs, res, boost::is_any_of(", "), boost::token_compress_on);
            bool found_me = false;
            for (std::size_t i = 0; i < strs.size(); ++i) {
                if (strs.at(i).find("average") != std::string::npos) {
                    if ((i + 3) <= strs.size()) {
                        load_factor.at(0) =
                            std::atof(strs.at(i + 1).c_str()) / (double)(processor_count);
                        load_factor.at(1) =
                            std::atof(strs.at(i + 2).c_str()) / (double)processor_count;
                        load_factor.at(2) =
                            std::atof(strs.at(i + 3).c_str()) / (double)processor_count;
                        found_me = true;
                        break;
                    }
                }
            }
            if (found_me == false) {
                diag.level = Level::Type::ERROR;
                diag.message = Diagnostic::Message::DROPPING_PACKETS;
                diag.description = "Unable to process string: " + res;
                diag.update_count++;
                return diag;
            }
        }
        catch (const std::exception &e) {
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::DROPPING_PACKETS;
            diag.description =
                "Unable to process string: " + res + " with result: " + std::string(e.what());
            diag.update_count++;
            return diag;
        }
    }
    diag.level = Level::Type::INFO;
    diag.message = Diagnostic::Message::NOERROR;
    diag.description = "Updated.";
    diag.update_count++;
    return diag;
}
Architecture::Type ResourceMonitor::read_device_architecture() {
    // Try 1: Read command: "uname -m"
    {
        std::string cmd = "uname -m";
        std::string result = exec(cmd.c_str(), true);
        std::size_t found_x86_64 = result.find("x86_64");
        std::size_t found_armv7l = result.find("armv7l");
        std::size_t found_aarch64 = result.find("aarch64");
        if (found_x86_64 != std::string::npos) {
            return Architecture::Type::X86_64;
        }
        else if (found_armv7l != std::string::npos) {
            return Architecture::Type::ARMV7L;
        }
        else if (found_aarch64 != std::string::npos) {
            return Architecture::Type::AARCH64;
        }
        else {
            logger->log_warn("Unexpected result of command: " + cmd + " result: " + result);
        }
    }
    return Architecture::Type::UNKNOWN;
}
std::string ResourceMonitor::pretty(ResourceInfo info) {
    std::string str = "--- Resource Monitor Info ---\n";
    str += "\tArchitecture: " + Architecture::ArchitectureString(architecture) +
           " Processor Count: " + std::to_string(processor_count) + "\n";
    str += "\tPID: " + std::to_string(info.pid) + "\n";
    str += "\tRAM: " + std::to_string(info.ram_perc) + "%\n";
    str += "\tCPU: " + std::to_string(info.cpu_perc) + "%\n";
    str += "\tDisk: " + std::to_string(info.disk_perc) + "%\n";

    return str;
}
std::string ResourceMonitor::exec(const char *cmd, bool wait_for_result) {
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
        return result;
    }
    catch (const std::exception &e) {
        std::string tempstr = "popen() failed with command: " + std::string(cmd) +
                              " and exception: " + std::string(e.what());
        logger->log_error(tempstr);
        return "";
    }
}