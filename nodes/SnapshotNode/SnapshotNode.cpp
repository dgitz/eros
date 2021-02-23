#include "SnapshotNode.h"
bool kill_node = false;
SnapshotNode::SnapshotNode()
    : system_command_action_server(
          *n.get(),
          "SystemCommandAction",
          boost::bind(&SnapshotNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
SnapshotNode::~SnapshotNode() {
}
void SnapshotNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    if (t_msg->Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT) {
        std::vector<Diagnostic::DiagnosticDefinition> diag_list = process->createnew_snapshot();

        Level::Type max_level = Level::Type::DEBUG;
        for (std::size_t i = 0; i < diag_list.size(); ++i) {
            if (diag_list.at(i).level > max_level) {
                max_level = diag_list.at(i).level;
                diag = diag_list.at(i);
            }
        }
        if (max_level <= Level::Type::WARN) {
            logger->log_notice("Snap Completed");
            eros::command_state state;
            state.stamp = ros::Time::now();
            state.NodeName = node_name;
            state.CurrentCommand = t_msg->Command;
            state.State = 1;
            state.PercentComplete = 100.0;
            state.diag = convert(diag);
            commandstate_pub.publish(state);
        }
        else {
            logger->log_warn("Snap Failed");
            eros::command_state state;
            state.stamp = ros::Time::now();
            state.NodeName = node_name;
            state.CurrentCommand = t_msg->Command;
            state.State = 0;
            state.PercentComplete = 0.0;
            state.diag = convert(diag);
            commandstate_pub.publish(state);
        }
    }
    else {
        diag =
            process->update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                       Level::Type::WARN,
                                       Diagnostic::Message::DROPPING_PACKETS,
                                       "Received unsupported Command: " +
                                           Command::CommandString((Command::Type)t_msg->Command));
    }
    logger->log_diagnostic(diag);
}
void SnapshotNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
}
bool SnapshotNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                           eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool SnapshotNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SnapshotProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }

    process->initialize(get_basenodename(),
                        get_nodename(),
                        get_hostname(),
                        DIAGNOSTIC_SYSTEM,
                        DIAGNOSTIC_SUBSYSTEM,
                        DIAGNOSTIC_COMPONENT,
                        logger);
    std::vector<Diagnostic::DiagnosticType> diagnostic_types;
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SOFTWARE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::DATA_STORAGE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::SYSTEM_RESOURCE);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::COMMUNICATIONS);
    process->enable_diagnostics(diagnostic_types);
    process->set_architecture(resource_monitor->get_architecture());
    diagnostic = process->finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    diagnostic = finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        return false;
    }
    if (diagnostic.level < Level::Type::WARN) {
        diagnostic.type = Diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = Level::Type::INFO;
        diagnostic.message = Diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    if (process->request_statechange(Node::State::INITIALIZED) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZED));
    }
    if (process->request_statechange(Node::State::RUNNING) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
Diagnostic::DiagnosticDefinition SnapshotNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string param_mode = node_name + "/Mode";
    std::string mode;
    if (n->getParam(param_mode, mode) == false) {
        diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          Diagnostic::Message::INITIALIZING_ERROR,
                                          "Mode Not defined.");
        logger->log_diagnostic(diag);
        return diag;
    }
    process->set_mode(SnapshotProcess::ModeType(mode));

    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition SnapshotNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "/" + node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SnapshotNode::changenodestate_service, this);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    std::string param_config_dir = node_name + "/Config_Directory";
    std::string config_dir;
    if (n->getParam(param_config_dir, config_dir) == false) {
        diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          Diagnostic::Message::INITIALIZING_ERROR,
                                          "Config_Directory Not defined.");
        logger->log_diagnostic(diag);
        return diag;
    }
    command_sub =
        n->subscribe<eros::command>("/SystemCommand", 10, &SnapshotNode::command_Callback, this);
    diag = process->load_config(config_dir + "/SnapshotConfig.xml");
    if (diag.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diag);
        return diag;
    }
    std::string commandstate_topic = "/SystemCommandState";
    commandstate_pub = n->advertise<eros::command_state>(commandstate_topic, 1);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    return diag;
}
bool SnapshotNode::run_loop1() {
    return true;
}
bool SnapshotNode::run_loop2() {
    return true;
}
bool SnapshotNode::run_loop3() {
    return true;
}
bool SnapshotNode::run_001hz() {
    return true;
}
bool SnapshotNode::run_01hz() {
    return true;
}
bool SnapshotNode::run_01hz_noisy() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    if ((deviceInfo.received == false) && (disable_device_client == false)) {
        logger->log_notice("Requesting Device Info");
        std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
        ros::ServiceClient client = n->serviceClient<eros::srv_device>(device_topic);
        eros::srv_device srv;
        if (client.call(srv)) {
            deviceInfo.received = true;
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::INFO,
                                              Diagnostic::Message::NOERROR,
                                              "Device Info Received.");
            logger->log_diagnostic(diag);
        }
        else {
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                              Level::Type::WARN,
                                              Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Device Info not received yet.");
            logger->log_diagnostic(diag);
        }
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool SnapshotNode::run_1hz() {
    std::vector<Diagnostic::DiagnosticDefinition> latest_diagnostics =
        process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(process->convert(latest_diagnostics.at(i)));
    }
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    if (process->get_nodestate() == Node::State::RESET) {
        process->reset();
        logger->log_notice("Node has Reset");
        if (process->request_statechange(Node::State::RUNNING) == false) {
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
        }
    }
    return true;
}
bool SnapshotNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    return true;
}
void SnapshotNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SnapshotNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing SnapshotNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "snapshot_node");
    SnapshotNode *node = new SnapshotNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&SnapshotNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}