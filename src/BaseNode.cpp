#include <eros/BaseNode.h>

void BaseNode::set_basenodename(std::string t_base_node_name) {
    base_node_name = t_base_node_name;
}
void BaseNode::initialize_firmware(uint8_t t_major_version,
                                   uint8_t t_minor_version,
                                   uint8_t t_build_number,
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
    heartbeat.BaseNodeName = base_node_name;
    heartbeat.NodeName = node_name;
    host_name[1023] = '\0';
    gethostname(host_name, 1023);
    rand_delay_sec = (double)(rand() % 2000 - 1000) / 1000.0;

    diagnostic = read_baselaunchparameters();
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<eros::heartbeat>(heartbeat_topic, 1);
    heartbeat.stamp = ros::Time::now();
    heartbeat.NodeState = (uint8_t)Node::State::INITIALIZING;
    heartbeat_pub.publish(heartbeat);
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
    std::string param_verbosity_level = node_name + "/verbosity_level";
    if (n->getParam(param_verbosity_level, verbosity_level) == false) {
        diagnostic.type = Diagnostic::DiagnosticType::DATA_STORAGE;
        diagnostic.level = Level::Type::ERROR;
        diagnostic.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Missing Parameter: verbosity_level. Exiting.";
        diagnostic = diag;
        return diag;
    }
    else {
        logger = new Logger(verbosity_level, node_name);
        logger_initialized = true;
    }
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
    std::string param_require_pps_to_start = node_name + "/require_pps_to_start";
    if (n->getParam(param_require_pps_to_start, require_pps_to_start) == false) {
        diagnostic.type = Diagnostic::DiagnosticType::DATA_STORAGE;
        diagnostic.level = Level::Type::ERROR;
        diagnostic.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Missing Parameter: require_pps_to_start. Exiting.";
        diagnostic = diag;
        logger->log_diagnostic(diag);
    }
    double max_rate = 0.0;
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
        logger->log_info("Node State: " + Node::NodeStateString(node_state));
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
void BaseNode::base_cleanup() {
    heartbeat.NodeState = (uint8_t)Node::State::FINISHED;
    heartbeat.stamp = ros::Time::now();
    heartbeat_pub.publish(heartbeat);
}