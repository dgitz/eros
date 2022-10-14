#include <eros/DiagnosticNode/DiagnosticNode.h>
using namespace eros;
using namespace eros_nodes;
bool kill_node = false;
DiagnosticNode::DiagnosticNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&DiagnosticNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
DiagnosticNode::~DiagnosticNode() {
}
void DiagnosticNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    (void)goal;
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
}
void DiagnosticNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    (void)t_msg;
}
void DiagnosticNode::diagnostic_Callback(const eros::diagnostic::ConstPtr &t_msg) {
    eros::diagnostic eros_diag = BaseNodeProcess::convert_fromptr(t_msg);
    Diagnostic::DiagnosticDefinition diag = process->convert(eros_diag);
    process->new_external_diagnostic(diag);
}
bool DiagnosticNode::system_diagnostics_service(eros::srv_get_diagnostics::Request &req,
                                                eros::srv_get_diagnostics::Response &res) {
    if ((req.MinLevel == 0) && (req.DiagnosticType == 0)) {
        for (uint8_t i = 1; i < (uint8_t)(Diagnostic::DiagnosticType::END_OF_LIST); ++i) {
            if ((Diagnostic::DiagnosticType)i == Diagnostic::DiagnosticType::UNKNOWN_TYPE) {
                continue;
            }
            res.diag_list.push_back(BaseNodeProcess::convert(
                process->get_worst_diagnostic((Diagnostic::DiagnosticType)(i))));
        }
        logger->log_debug("Fulfilled System Diagnostics Service.");
        return true;
    }
    else {
        logger->log_warn("Unsupported Srv Query: " + std::to_string(req.MinLevel) + " " +
                         std::to_string(req.DiagnosticType));
        return false;
    }
}
bool DiagnosticNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                             eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool DiagnosticNode::start() {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new DiagnosticNodeProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    enable_ready_to_arm_pub(true);
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
    diagnostic = rescan_nodes();
    if (diagnostic.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diagnostic);
        return false;
    }
    if (process->request_statechange(Node::State::INITIALIZING) == false) {
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZING));
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
Diagnostic::DiagnosticDefinition DiagnosticNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition DiagnosticNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_system_diagnostics_topic = get_robotnamespace() + "/srv_system_diagnostics";
    system_diagnostics_srv = n->advertiseService(
        srv_system_diagnostics_topic, &DiagnosticNode::system_diagnostics_service, this);
    std::string srv_nodestate_topic = node_name + "/srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &DiagnosticNode::changenodestate_service, this);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    return diag;
}
bool DiagnosticNode::run_loop1() {
    return true;
}
bool DiagnosticNode::run_loop2() {
    Diagnostic::DiagnosticDefinition diag = rescan_nodes();
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    return true;
}
bool DiagnosticNode::run_loop3() {
    return true;
}
bool DiagnosticNode::run_001hz() {
    return true;
}
bool DiagnosticNode::run_01hz() {
    return true;
}
bool DiagnosticNode::run_01hz_noisy() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    logger->log_debug(process->pretty());
    return true;
}
bool DiagnosticNode::run_1hz() {
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
bool DiagnosticNode::run_10hz() {
    Diagnostic::DiagnosticDefinition diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
Diagnostic::DiagnosticDefinition DiagnosticNode::rescan_nodes() {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    std::size_t found_new_subscribers = 0;
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::vector<std::string> diagnostic_list;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end();
         it++) {
        const ros::master::TopicInfo &info = *it;
        std::size_t found = info.name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }

        if (info.datatype == "eros/diagnostic") {
            diagnostic_list.push_back(info.name);
        }
    }
    std::vector<std::string> new_diagnostic_topics_to_subscribe = diagnostic_list;
    diag = process->update_topiclist(new_diagnostic_topics_to_subscribe);
    if (diag.level >= Level::Type::WARN) {
        logger->log_diagnostic(diag);
        return diag;
    }
    for (std::size_t i = 0; i < new_diagnostic_topics_to_subscribe.size(); ++i) {
        ros::Subscriber sub =
            n->subscribe<eros::diagnostic>(new_diagnostic_topics_to_subscribe.at(i),
                                           50,
                                           &DiagnosticNode::diagnostic_Callback,
                                           this);
        diagnostic_subs.push_back(sub);
    }
    found_new_subscribers = new_diagnostic_topics_to_subscribe.size();
    if (found_new_subscribers > 0) {
        logger->log_info("Rescanned and found " + std::to_string(found_new_subscribers) +
                         " new things to subscribe to.");
    }
    else {
        logger->log_info("Rescanned and found no new things to subscribe to.");
    }
    return diag;
}
void DiagnosticNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void DiagnosticNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing DiagnosticNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "diagnostic_node");
    DiagnosticNode *node = new DiagnosticNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&DiagnosticNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
