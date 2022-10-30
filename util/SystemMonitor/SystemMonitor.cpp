// No practical way to this file due to screen rendering.
// LCOV_EXCL_START
#include <eros/SystemMonitor/SystemMonitor.h>
using namespace eros;
bool kill_node = false;
SystemMonitor::SystemMonitor()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&SystemMonitor::system_commandAction_Callback, this, _1),
          false) {
    filter_list.insert(std::make_pair("rostopic", true));
    system_command_action_server.start();
}
SystemMonitor::~SystemMonitor() {
}
void SystemMonitor::heartbeat_Callback(const eros::heartbeat::ConstPtr &t_msg) {
    Diagnostic::DiagnosticDefinition diag = process->new_heartbeatmessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitor::loadfactor_Callback(const eros::loadfactor::ConstPtr &msg) {
    Diagnostic::DiagnosticDefinition diag = process->new_loadfactormessage(msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitor::resourceAvailable_Callback(const eros::resource::ConstPtr &msg) {
    Diagnostic::DiagnosticDefinition diag = process->new_resourceavailablemessage(msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitor::resourceUsed_Callback(const eros::resource::ConstPtr &msg) {
    Diagnostic::DiagnosticDefinition diag = process->new_resourceusedmessage(msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
void SystemMonitor::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    eros::system_commandResult system_commandResult_;
    system_command_action_server.setAborted(system_commandResult_);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::WARN,
                                      Diagnostic::Message::DROPPING_PACKETS,
                                      "Received unsupported CommandAction: " +
                                          Command::CommandString((Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void SystemMonitor::command_Callback(const eros::command::ConstPtr &t_msg) {
    eros::command cmd = BaseNodeProcess::convert_fromptr(t_msg);
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    diag = process->update_diagnostic(
        Diagnostic::DiagnosticType::COMMUNICATIONS,
        Level::Type::WARN,
        Diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " + Command::CommandString((Command::Type)cmd.Command));
    logger->log_diagnostic(diag);
}
bool SystemMonitor::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                            eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool SystemMonitor::start() {
    set_no_launch_enabled(true);
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SystemMonitorProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    diagnostic = read_launchparameters();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
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
    diagnostic_types.push_back(Diagnostic::DiagnosticType::REMOTE_CONTROL);
    process->enable_diagnostics(diagnostic_types);
    diagnostic = process->finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    diagnostic = finish_initialization();
    if (diagnostic.level > Level::Type::WARN) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return false;
        // LCOV_EXCL_STOP
    }
    get_logger()->set_logverbosity(Level::Type::DEBUG);
    if (diagnostic.level < Level::Type::WARN) {
        diagnostic.type = Diagnostic::DiagnosticType::SOFTWARE;
        diagnostic.level = Level::Type::INFO;
        diagnostic.message = Diagnostic::Message::NOERROR;
        diagnostic.description = "Node Configured.  Initializing.";
        get_logger()->log_diagnostic(diagnostic);
    }
    if (process->request_statechange(Node::State::INITIALIZING) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZING));
        // LCOV_EXCL_STOP
    }
    diagnostic = rescan_nodes();
    if (diagnostic.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diagnostic);
        return false;
    }
    if (process->request_statechange(Node::State::INITIALIZED) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::INITIALIZED));
        // LCOV_EXCL_STOP
    }
    if (process->request_statechange(Node::State::RUNNING) == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        logger->log_warn("Unable to Change State to: " +
                         Node::NodeStateString(Node::State::RUNNING));
        // LCOV_EXCL_STOP
    }
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    status = true;
    return status;
}
Diagnostic::DiagnosticDefinition SystemMonitor::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    command_sub = n->subscribe<eros::command>(
        get_robotnamespace() + "SystemCommand", 10, &SystemMonitor::command_Callback, this);
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitor::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &SystemMonitor::changenodestate_service, this);
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Comms Ready.");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");
    set_loop1_rate(1.0);
    set_loop2_rate(0.2);
    set_ros_rate(20.0);
    return diag;
}
bool SystemMonitor::run_loop1() {
    return true;
}
bool SystemMonitor::run_loop2() {
    Diagnostic::DiagnosticDefinition diag = rescan_nodes();
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    return true;
}
bool SystemMonitor::run_loop3() {
    return true;
}
bool SystemMonitor::run_001hz() {
    return true;
}
bool SystemMonitor::run_01hz() {
    return true;
}
bool SystemMonitor::run_01hz_noisy() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool SystemMonitor::run_1hz() {
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
            // No practical way to unit test
            // LCOV_EXCL_START
            diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                              Level::Type::ERROR,
                                              Diagnostic::Message::DEVICE_NOT_AVAILABLE,
                                              "Not able to Change Node State to Running.");
            logger->log_diagnostic(diag);
            // LCOV_EXCL_STOP
        }
    }
    return true;
}
bool SystemMonitor::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    process->update(0.1, ros::Time::now().toSec());
    if (process->shouldKill() == true) {
        kill_node = true;
    }
    return true;
}
void SystemMonitor::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SystemMonitor::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
// No practical way to unit test
// LCOV_EXCL_START
void signalinterrupt_handler(int sig) {
    printf("Killing SystemMonitor with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}

// LCOV_EXCL_STOP
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "system_monitor");
    SystemMonitor *node = new SystemMonitor();
    bool status = node->start();
    if (status == false) {
        // No practical way to unit test
        // LCOV_EXCL_START
        return EXIT_FAILURE;
        // LCOV_EXCL_STOP
    }
    std::thread thread(&SystemMonitor::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
Diagnostic::DiagnosticDefinition SystemMonitor::rescan_nodes() {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    std::size_t found_new_subscribers = 0;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::vector<std::string> heartbeat_list;
    std::vector<std::string> resource_used_list;
    std::vector<std::string> loadfactor_list;
    std::vector<std::string> resource_available_list;
    std::vector<std::string> eros_name_list;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end();
         it++) {
        const ros::master::TopicInfo &info = *it;
        std::size_t found = info.name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }

        if (info.datatype == "eros/heartbeat") {
            if (info.name.rfind(get_robotnamespace(), 0) == 0) {
                if (heartbeat_subs.find(info.name) == heartbeat_subs.end()) {
                    heartbeat_list.push_back(info.name);
                    eros_name_list.push_back(info.name);
                }
            }
        }
        if (info.datatype == "eros/resource") {
            if (info.name.find("resource_available") != std::string::npos) {
                if (resourceavailable_subs.find(info.name) == resourceavailable_subs.end()) {
                    resource_available_list.push_back(info.name);
                    eros_name_list.push_back(info.name);
                }
            }
            if (info.name.find("resource_used") != std::string::npos) {
                if (resourceused_subs.find(info.name) == resourceused_subs.end()) {
                    resource_used_list.push_back(info.name);
                    eros_name_list.push_back(info.name);
                }
            }
        }
        if (info.datatype == "eros/loadfactor") {
            if (info.name.rfind(get_robotnamespace(), 0) == 0) {
                loadfactor_list.push_back(info.name);
                eros_name_list.push_back(info.name);
            }
        }
    }
    for (std::size_t i = 0; i < heartbeat_list.size(); ++i) {
        ros::Subscriber sub = n->subscribe<eros::heartbeat>(
            heartbeat_list.at(i), 50, &SystemMonitor::heartbeat_Callback, this);
        heartbeat_subs.insert(std::pair<std::string, ros::Subscriber>(heartbeat_list.at(i), sub));
    }
    for (std::size_t i = 0; i < resource_available_list.size(); ++i) {
        ros::Subscriber sub = n->subscribe<eros::resource>(
            resource_available_list.at(i), 50, &SystemMonitor::resourceAvailable_Callback, this);
        resourceavailable_subs.insert(
            std::pair<std::string, ros::Subscriber>(resource_available_list.at(i), sub));
    }
    for (std::size_t i = 0; i < resource_used_list.size(); ++i) {
        ros::Subscriber sub = n->subscribe<eros::resource>(
            resource_used_list.at(i), 50, &SystemMonitor::resourceUsed_Callback, this);
        resourceused_subs.insert(
            std::pair<std::string, ros::Subscriber>(resource_used_list.at(i), sub));
    }
    for (std::size_t i = 0; i < loadfactor_list.size(); ++i) {
        ros::Subscriber sub = n->subscribe<eros::loadfactor>(
            loadfactor_list.at(i), 50, &SystemMonitor::loadfactor_Callback, this);
        loadfactor_subs.insert(std::pair<std::string, ros::Subscriber>(loadfactor_list.at(i), sub));
    }

    ros::V_string nodes;
    ros::master::getNodes(nodes);
    std::vector<std::string> generic_node_list;
    for (ros::V_string::iterator it = nodes.begin(); it != nodes.end(); it++) {
        const std::string &_node_name = *it;
        std::size_t found = _node_name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }
        bool add_me = true;
        if (_node_name.rfind(get_robotnamespace(), 0) != 0) {
            add_me = false;
        }
        if (add_me == true) {
            std::map<std::string, bool>::iterator filter_it = filter_list.begin();
            while (filter_it != filter_list.end()) {
                if (filter_it->second == true) {
                    if (_node_name.find(filter_it->first) != std::string::npos) {
                        add_me = false;
                    }
                }
                filter_it++;
            }
        }
        std::vector<std::string>::iterator itGenericNode =
            find(genericNodeList.begin(), genericNodeList.end(), _node_name);
        if (itGenericNode != genericNodeList.end()) {
            add_me = false;
        }
        else {
            std::vector<std::string>::iterator itErosNode =
                find(eros_name_list.begin(), eros_name_list.end(), _node_name);

            if (itErosNode != eros_name_list.end()) {
                logger->log_warn("Not adding: " + _node_name + " Since its an eros node.");
                add_me = false;
            }
        }

        if (add_me == true) {
            generic_node_list.push_back(_node_name);
            genericNodeList.push_back(_node_name);
        }
    }
    for (auto genericNode : generic_node_list) {
        diag = process->update_genericNode("", genericNode, ros::Time::now().toSec());
        if (diag.level > Level::Type::NOTICE) {
            logger->log_error("Unable to Update Generic Node: " + genericNode);
            return diag;
        }
    }

    found_new_subscribers = eros_name_list.size() + generic_node_list.size();
    if (found_new_subscribers > 0) {
        logger->log_notice("Rescanned and found " + std::to_string(found_new_subscribers) +
                           " new things to subscribe to.");
    }
    else {
        logger->log_notice("Rescanned and found no new things to subscribe to.");
    }
    return diag;
}
// LCOV_EXCL_STOP