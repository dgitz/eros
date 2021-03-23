#include "SafetyNode.h"
bool kill_node = false;

SafetyNode::SafetyNode()
    : system_command_action_server(
          *n.get(),
          get_hostname() + "_" + SafetyNode::BASE_NODE_NAME + "_SystemCommand",
          boost::bind(&SafetyNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
SafetyNode::~SafetyNode() {
}
void SafetyNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
}
void SafetyNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    process->new_commandmsg(BaseNodeProcess::convert_fromptr(t_msg));
}
bool SafetyNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool SafetyNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SafetyNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    disable_armedstate_sub();
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
    diagnostic_types.push_back(Diagnostic::DiagnosticType::REMOTE_CONTROL);
    diagnostic_types.push_back(Diagnostic::DiagnosticType::COMMUNICATIONS);
    process->enable_diagnostics(diagnostic_types);
    process->finish_initialization();
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
Diagnostic::DiagnosticDefinition SafetyNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition SafetyNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "/" + node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SafetyNode::changenodestate_service, this);
    armedstate_pub = n->advertise<eros::armed_state>("/ArmedState", 2);
    command_sub =
        n->subscribe<eros::command>("/SystemCommand", 10, &SafetyNode::command_Callback, this);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    // Read Ready To Arm Topics
    std::vector<std::string> topics;
    std::vector<ArmDisarmMonitor::Type> types;

    uint16_t counter = 0;
    bool valid_arm_topic = true;
    while (valid_arm_topic == true) {
        char param_topic[512];
        sprintf(param_topic, "%s/Default_ReadyToArmTopic_%03d", node_name.c_str(), counter);
        std::string topic;
        if (n->getParam(param_topic, topic) == true) {
            logger->log_notice("Subscribing to Default ReadyToArm Topic: " + topic);
            topics.push_back(topic);
            types.push_back(ArmDisarmMonitor::Type::DEFAULT);
        }
        else {
            valid_arm_topic = false;
            break;
        }
        counter++;
    }
    valid_arm_topic = true;
    counter = 0;
    while (valid_arm_topic == true) {
        char param_topic[512];
        sprintf(param_topic, "%s/Simple_ReadyToArmTopic_%03d", node_name.c_str(), counter);
        std::string topic;
        if (n->getParam(param_topic, topic) == true) {
            logger->log_notice("Subscribing to Simple ReadyToArm Topic: " + topic);
            topics.push_back(topic);
            types.push_back(ArmDisarmMonitor::Type::SIMPLE);
        }
        else {
            valid_arm_topic = false;
            break;
        }
        counter++;
    }
    if (process->initialize_readytoarm_monitors(topics, types) == false) {
        diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                          Level::Type::ERROR,
                                          Diagnostic::Message::INITIALIZING_ERROR,
                                          "Unable to initialize Ready To Arm Monitors.");
        logger->log_diagnostic(diag);
        return diag;
    }
    if (process->init_ros(n) == false) {
        diag = process->update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                          Level::Type::ERROR,
                                          Diagnostic::Message::INITIALIZING_ERROR,
                                          "Unable to initialize ROS in Safety Node Process");
        logger->log_diagnostic(diag);
        return diag;
    }
    return diag;
}
bool SafetyNode::run_loop1() {
    return true;
}
bool SafetyNode::run_loop2() {
    return true;
}
bool SafetyNode::run_loop3() {
    return true;
}
bool SafetyNode::run_001hz() {
    return true;
}
bool SafetyNode::run_01hz() {
    return true;
}
bool SafetyNode::run_01hz_noisy() {
    {
        std::vector<std::string> cannotarm_reasons = process->get_cannotarm_reasons();
        for (auto reason : cannotarm_reasons) { logger->log_warn(reason); }
    }
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool SafetyNode::run_1hz() {
    std::vector<Diagnostic::DiagnosticDefinition> latest_diagnostics =
        process->get_latest_diagnostics();
    for (std::size_t i = 0; i < latest_diagnostics.size(); ++i) {
        logger->log_diagnostic(latest_diagnostics.at(i));
        diagnostic_pub.publish(process->convert(latest_diagnostics.at(i)));
    }
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    if (process->get_nodestate() == Node::State::RESET) {
        base_reset();
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
bool SafetyNode::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    Diagnostic::DiagnosticDefinition diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }

    armedstate_pub.publish(process->convert(process->get_armed_state()));
    return true;
}
void SafetyNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SafetyNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing SafetyNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "safety_node");
    SafetyNode *node = new SafetyNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&SafetyNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
