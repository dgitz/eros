#include <eros/BaseNode.h>
void BaseNode::set_basenodename(std::string t_base_node_name) {
    base_node_name = t_base_node_name;
}
void BaseNode::initialize_firmware(uint16_t t_major_version,
                                   uint16_t t_minor_version,
                                   uint16_t t_build_number,
                                   std::string t_description) {
    firmware_version.MajorVersion = t_major_version;
    firmware_version.MinorVersion = t_minor_version;
    firmware_version.BuildNumber = t_build_number;
    firmware_version.Description = t_description;
}
void BaseNode::initialize_diagnostic(System::MainSystem t_system,
                                     System::SubSystem t_subsystem,
                                     System::Component t_component) {
    diagnostic.system = t_system;
    diagnostic.subsystem = t_subsystem;
    diagnostic.component = t_component;
}
Diagnostic::DiagnosticDefinition BaseNode::preinitialize_basenode(int argc, char** argv) {
    logger_initialized = false;
    require_pps_to_start = false;
    pps_received = false;
    ros::init(argc, argv, base_node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    boot_time = ros::Time::now();
    diagnostic.type = Diagnostic::DiagnosticType::SOFTWARE;
    diagnostic.level = Level::Type::INFO;
    diagnostic.message = Diagnostic::Message::INITIALIZING;
    diagnostic.description = "Node Initializing.";

    host_name[1023] = '\0';
    gethostname(host_name, 1023);
    heartbeat.HostName = host_name;
    heartbeat.BaseNodeName = base_node_name;
    heartbeat.NodeName = node_name;
    rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;

    diagnostic = read_baselaunchparameters();
    if (diagnostic.level > Level::Type::WARN) {
        if (logger_initialized == true) {
            logger->log_diagnostic(diagnostic);
        }
        else {
            printf("[%s ERROR]: %s\n", node_name.c_str(), diagnostic.description.c_str());
        }
    }
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<eros::heartbeat>(heartbeat_topic, 1);
    heartbeat.stamp = ros::Time::now();
    heartbeat.NodeState = (uint8_t)Node::State::INITIALIZING;
    heartbeat_pub.publish(heartbeat);

    std::string srv_firmware_topic = "/" + node_name + "/srv_firmware";
    firmware_srv = n->advertiseService(srv_firmware_topic, &BaseNode::firmware_service, this);

    std::string srv_loggerlevel_topic = "/" + node_name + "/srv_loggerlevel";
    logger_level_srv =
        n->advertiseService(srv_loggerlevel_topic, &BaseNode::loggerlevel_service, this);

    std::string srv_diagnostics_topic = "/" + node_name + "/srv_diagnostics";
    diagnostics_srv =
        n->advertiseService(srv_diagnostics_topic, &BaseNode::diagnostics_service, this);

    if (diagnostic.level > Level::Type::WARN) {
        if (logger_initialized == true) {
            logger->log_diagnostic(diagnostic);
        }
        else {
            printf("[%s] Could not complete pre-initialization. Exiting.\n", node_name.c_str());
        }
    }
    return diagnostic;
}

Diagnostic::DiagnosticDefinition BaseNode::read_baselaunchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    loop1_enabled = false;
    loop2_enabled = false;
    loop3_enabled = false;
    if (no_launch_enabled == true) {
        logger = new Logger("NOTICE", node_name);
        logger_initialized = true;
    }
    else {
        std::string param_verbosity_level = node_name + "/verbosity_level";
        if (n->getParam(param_verbosity_level, verbosity_level) == false) {
            diag.type = Diagnostic::DiagnosticType::DATA_STORAGE;
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::INITIALIZING_ERROR;
            diag.description = "Missing Parameter: verbosity_level. Exiting.";
            diagnostic = diag;
            return diag;
        }
        else {
            logger = new Logger(verbosity_level, node_name);
            logger->log_notice("Initialized.");
            logger_initialized = true;
        }
    }
    if (no_launch_enabled == true) {
        std::string param_startup_delay = node_name + "/startup_delay";
        double startup_delay = 0.0;
        if (n->getParam(param_startup_delay, startup_delay) == false) {
            logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
        }
        else {
            char tempstr[128];
            sprintf(tempstr, "Using Parameter: startup_delay = %4.2f sec.", startup_delay);
            logger->log_notice(std::string(tempstr));
        }
        ros::Duration(startup_delay).sleep();
    }
    if (no_launch_enabled == true) {}
    else {
        std::string param_require_pps_to_start = node_name + "/require_pps_to_start";
        if (n->getParam(param_require_pps_to_start, require_pps_to_start) == false) {
            diag.type = Diagnostic::DiagnosticType::DATA_STORAGE;
            diag.level = Level::Type::ERROR;
            diag.message = Diagnostic::Message::INITIALIZING_ERROR;
            diag.description = "Missing Parameter: require_pps_to_start. Exiting.";
            diagnostic = diag;
            logger->log_diagnostic(diag);
        }
    }
    double max_rate = 0.0;
    if (no_launch_enabled == true) {
        loop1_enabled = true;
        loop1_rate = 1.0;
        loop2_enabled = true;
        loop2_rate = 5.0;
        loop3_enabled = false;
        max_rate = loop2_rate;
    }
    else {
        last_01hz_timer = ros::Time::now();
        last_01hz_noisy_timer = ros::Time::now();
        last_1hz_timer = ros::Time::now();
        last_10hz_timer = ros::Time::now();
        std::string param_loop1_rate = node_name + "/loop1_rate";
        if (n->getParam(param_loop1_rate, loop1_rate) == false) {
            logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
            loop1_enabled = false;
        }
        else {
            last_loop1_timer = ros::Time::now();
            loop1_enabled = true;
            if (loop1_rate > max_rate) {
                max_rate = loop1_rate;
            }
        }

        std::string param_loop2_rate = node_name + "/loop2_rate";
        if (n->getParam(param_loop2_rate, loop2_rate) == false) {
            logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
            loop2_enabled = false;
        }
        else {
            last_loop2_timer = ros::Time::now();
            loop2_enabled = true;
            if (loop2_rate > max_rate) {
                max_rate = loop2_rate;
            }
        }

        std::string param_loop3_rate = node_name + "/loop3_rate";
        if (n->getParam(param_loop3_rate, loop3_rate) == false) {
            logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
            loop3_enabled = false;
        }
        else {
            last_loop3_timer = ros::Time::now();
            loop3_enabled = true;
            if (loop3_rate > max_rate) {
                max_rate = loop3_rate;
            }
        }
    }
    ros_rate = max_rate * 4.0;
    if (ros_rate > 100.0) {
        ros_rate = 100.0;
    }
    if (ros_rate <= 1.0) {
        ros_rate = 20.0;
    }
    char tempstr[512];
    sprintf(tempstr, "Running Node at Rate: %4.2f Hz.", ros_rate);
    logger->log_notice(std::string(tempstr));
    diagnostic = diag;
    return diag;
}

bool BaseNode::update(Node::State node_state) {
    bool ok_to_run = false;
    if (require_pps_to_start == false) {
        ok_to_run = true;
    }
    else if ((require_pps_to_start == true) and (pps_received == true)) {
        ok_to_run = true;
    }
    if (ok_to_run == false) {
        ros::Duration d(1.0);
        d.sleep();
        ros::spinOnce();
        logger->log_notice("Waiting on PPS To Start.");
        return true;
    }
    ros::Rate r(ros_rate);
    r.sleep();
    ros::spinOnce();
    double mtime;
    mtime = measure_time_diff(ros::Time::now(), last_001hz_timer);

    if (mtime >= 100.0) {
        run_001hz();
        last_001hz_timer = ros::Time::now();
    }
    mtime = measure_time_diff(ros::Time::now(), last_01hz_noisy_timer);
    if (mtime >= 10.0 + rand_delay_sec) {
        rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;
        run_01hz_noisy();
        last_01hz_noisy_timer = ros::Time::now();
    }
    mtime = measure_time_diff(ros::Time::now(), last_01hz_timer);
    if (mtime >= 10.0) {
        run_01hz();
        last_01hz_timer = ros::Time::now();
    }
    mtime = measure_time_diff(ros::Time::now(), last_1hz_timer);
    if (mtime >= 1.0) {
        run_1hz();
        last_1hz_timer = ros::Time::now();
    }
    mtime = measure_time_diff(ros::Time::now(), last_10hz_timer);
    if (mtime >= 0.1) {
        run_10hz();
        heartbeat.NodeState = (uint8_t)node_state;
        heartbeat.stamp = ros::Time::now();
        heartbeat_pub.publish(heartbeat);
        last_10hz_timer = ros::Time::now();
    }

    if (loop1_enabled == true) {
        mtime = measure_time_diff(ros::Time::now(), last_loop1_timer);
        if (mtime >= (1.0 / loop1_rate)) {
            run_loop1();
            last_loop1_timer = ros::Time::now();
        }
    }
    if (loop2_enabled == true) {
        mtime = measure_time_diff(ros::Time::now(), last_loop2_timer);
        if (mtime >= (1.0 / loop2_rate)) {
            run_loop2();
            last_loop2_timer = ros::Time::now();
        }
    }
    if (loop3_enabled == true) {
        mtime = measure_time_diff(ros::Time::now(), last_loop3_timer);
        if (mtime >= (1.0 / loop3_rate)) {
            run_loop3();
            last_loop3_timer = ros::Time::now();
        }
    }
    return ros::ok();
}

void BaseNode::new_ppsmsg(const std_msgs::Bool::ConstPtr& t_msg) {
    if (t_msg->data == true) {
        pps_received = true;
    }
}
bool BaseNode::firmware_service(eros::srv_firmware::Request& req,
                                eros::srv_firmware::Response& res) {
    (void)req;  // No req information needed
    res.BaseNodeName = base_node_name;
    res.NodeName = node_name;
    res.MajorRelease = firmware_version.MajorVersion;
    res.MinorRelease = firmware_version.MinorVersion;
    res.BuildNumber = firmware_version.BuildNumber;
    res.Description = firmware_version.Description;
    return true;
}
bool BaseNode::loggerlevel_service(eros::srv_logger_level::Request& req,
                                   eros::srv_logger_level::Response& res) {
    Level::Type newLevel = Level::LevelType(req.LoggerLevel);
    if (newLevel == Level::Type::UNKNOWN) {
        res.Response = "Unsupported Logger Level: " + req.LoggerLevel;
        return false;
    }
    else if (logger == nullptr) {
        res.Response = "Logger is uninitialized.";
        return false;
    }
    else {
        logger->set_logverbosity(newLevel);
        res.Response = "Changed Logger Level to: " + req.LoggerLevel;
        return true;
    }
}
bool BaseNode::diagnostics_service(eros::srv_get_diagnostics::Request& req,
                                   eros::srv_get_diagnostics::Response& res) {
    // Ignore req for now
    std::vector<Diagnostic::DiagnosticDefinition> diag_list = current_diagnostics;
    for (std::size_t i = 0; i < diag_list.size(); ++i) {
        eros::diagnostic diag = convert(diag_list.at(i));
        bool add_me = false;
        if ((req.MinLevel == 0) || (req.DiagnosticType == 0)) {
            add_me = true;
        }
        else if ((diag.Level >= req.MinLevel)) {
            add_me = true;
        }
        else if (diag.DiagnosticType == req.DiagnosticType) {
            add_me = true;
        }
        if (add_me == true) {
            res.diag_list.push_back(diag);
        }
    }
    return true;
}
void BaseNode::base_cleanup() {
    for (int i = 0; i < 5; ++i) {
        heartbeat.NodeState = (uint8_t)Node::State::FINISHED;
        heartbeat.stamp = ros::Time::now();
        heartbeat_pub.publish(heartbeat);
    }
}
eros::diagnostic BaseNode::convert(Diagnostic::DiagnosticDefinition diag_def) {
    eros::diagnostic diag;
    diag.DeviceName = diag_def.device_name;
    diag.NodeName = diag_def.node_name;
    diag.System = (uint8_t)diag_def.system;
    diag.SubSystem = (uint8_t)diag_def.subsystem;
    diag.Component = (uint8_t)diag_def.component;
    diag.DiagnosticType = (uint8_t)diag_def.type;
    diag.Level = (uint8_t)diag_def.level;
    diag.DiagnosticMessage = (uint8_t)diag_def.message;
    diag.Description = diag_def.description;
    return diag;
}