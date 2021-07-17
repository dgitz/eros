#include <eros/MasterNode/MasterNode.h>
using namespace eros;
using namespace eros_nodes;
bool kill_node = false;
MasterNode::MasterNode()
    : system_command_action_server(
          *n.get(),
          read_robotnamespace() + "SystemCommandAction",
          boost::bind(&MasterNode::system_commandAction_Callback, this, _1),
          false) {
    system_command_action_server.start();
}
MasterNode::~MasterNode() {
}

void MasterNode::system_commandAction_Callback(const eros::system_commandGoalConstPtr &goal) {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    eros::system_commandResult result_;
    system_command_action_server.setAborted(result_);
    diag = process->update_diagnostic(
        Diagnostic::DiagnosticType::COMMUNICATIONS,
        Level::Type::WARN,
        Diagnostic::Message::DROPPING_PACKETS,
        "Received unsupported Command: " + Command::CommandString((Command::Type)goal->Command));
    logger->log_diagnostic(diag);
}
void MasterNode::command_Callback(const eros::command::ConstPtr &t_msg) {
    (void)t_msg;
}
bool MasterNode::changenodestate_service(eros::srv_change_nodestate::Request &req,
                                         eros::srv_change_nodestate::Response &res) {
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool MasterNode::device_service(eros::srv_device::Request &req, eros::srv_device::Response &res) {
    (void)req;  // Currently Unused
    (void)res;
    if (deviceInfo.received == true) {
        return true;
    }
    else {
        return false;
    }
}
bool MasterNode::start() {
    disable_device_client = true;
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new MasterNodeProcess();
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
    deviceInfo.received = true;

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
Diagnostic::DiagnosticDefinition MasterNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition MasterNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "srv_nodestate_change";
    nodestate_srv =
        n->advertiseService(srv_nodestate_topic, &MasterNode::changenodestate_service, this);
    std::string srv_device_topic = "srv_device";
    device_server_srv = n->advertiseService(srv_device_topic, &MasterNode::device_service, this);

    std::string resource_available_topic =
        get_robotnamespace() + get_hostname() + "/resource_available";
    resource_available_pub = n->advertise<eros::resource>(resource_available_topic, 1);

    std::string device_loadfactor_topic = get_robotnamespace() + get_hostname() + "/loadfactor";
    loadfactor_pub = n->advertise<eros::loadfactor>(device_loadfactor_topic, 5);

    diag = process->update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "Running");
    diag = process->update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                      Level::Type::INFO,
                                      Diagnostic::Message::NOERROR,
                                      "All Configuration Files Loaded.");

    resource_available_monitor = new ResourceMonitor(ResourceMonitor::Mode::DEVICE, diag, logger);
    diag = resource_available_monitor->init();
    if (diag.level > Level::Type::WARN) {
        logger->log_diagnostic(diag);
        return diag;
    }
    return diag;
}
bool MasterNode::run_loop1() {
    return true;
}
bool MasterNode::run_loop2() {
    return true;
}
bool MasterNode::run_loop3() {
    return true;
}
bool MasterNode::run_001hz() {
    return true;
}
bool MasterNode::run_01hz() {
    return true;
}
bool MasterNode::run_01hz_noisy() {
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    Diagnostic::DiagnosticDefinition diag = resource_available_monitor->update(10.0);
    logger->log_diagnostic(diag);
    if (diag.level <= Level::Type::WARN) {
        {
            eros::resource msg = convert(resource_available_monitor->get_resourceinfo());
            msg.Name = get_robotnamespace() + get_hostname();
            msg.stamp = ros::Time::now();
            resource_available_pub.publish(msg);
        }
        {
            eros::loadfactor msg;
            std::vector<double> load_factor = resource_available_monitor->get_load_factor();
            msg.stamp = ros::Time::now();
            msg.DeviceName = get_robotnamespace() + get_hostname();
            msg.loadfactor.push_back(load_factor.at(0));
            msg.loadfactor.push_back(load_factor.at(1));
            msg.loadfactor.push_back(load_factor.at(2));
            loadfactor_pub.publish(msg);
        }
    }
    return true;
}
bool MasterNode::run_1hz() {
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
bool MasterNode::run_10hz() {
    Diagnostic::DiagnosticDefinition diag = process->update(0.1, ros::Time::now().toSec());
    if (diag.level >= Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    update_diagnostics(process->get_diagnostics());
    update_ready_to_arm(process->get_ready_to_arm());
    return true;
}
void MasterNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void MasterNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing MasterNode with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    ros::init(argc, argv, "master_node");

    MasterNode *node = new MasterNode();
    bool status = node->start();
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&MasterNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
