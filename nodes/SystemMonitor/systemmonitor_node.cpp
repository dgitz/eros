#include "systemmonitor_node.h"
bool kill_node = false;

SystemMonitorNode::SystemMonitorNode() {
}
SystemMonitorNode::~SystemMonitorNode() {
}
bool SystemMonitorNode::start(int argc, char **argv) {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SystemMonitorProcess();
    set_basenodename(BASE_NODE_NAME);
    initialize_firmware(
        MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
    diagnostic = preinitialize_basenode(argc, argv);
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
    logger->disable_consoleprint();  // Disabling as System Monitor will use console window.
    status = init_screen();
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Unable to initialize screen");
    }
    diagnostic = rescan_nodes();
    if (diagnostic.level >= Level::Type::ERROR) {
        logger->log_diagnostic(diagnostic);
        return false;
    }
    return status;
}
Diagnostic::DiagnosticDefinition SystemMonitorNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    return diag;
}
bool SystemMonitorNode::run_loop1() {
    Diagnostic::DiagnosticDefinition diagnostic = process->update(.1, ros::Time::now().toSec());
    if (diagnostic.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diagnostic);
    }
    return true;
}
bool SystemMonitorNode::run_loop2() {
    Diagnostic::DiagnosticDefinition diag = rescan_nodes();
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
    return true;
}
bool SystemMonitorNode::run_loop3() {
    return true;
}
bool SystemMonitorNode::run_001hz() {
    return true;
}
bool SystemMonitorNode::run_01hz() {
    return true;
}
bool SystemMonitorNode::run_01hz_noisy() {
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool SystemMonitorNode::run_1hz() {
    return true;
}
bool SystemMonitorNode::run_10hz() {
    return true;
}
void SystemMonitorNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SystemMonitorNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    logger->enable_consoleprint();
    logger->log_notice("Closing System Monitor.");
    base_cleanup();
    delete process;
}

bool SystemMonitorNode::init_screen() {
    mousemask(ALL_MOUSE_EVENTS, NULL);
    initscr();
    clear();
    if (has_colors() == FALSE) {
        endwin();
        logger->enable_consoleprint();
        logger->log_error("Terminal does not support colors. Exiting.");
        return false;
    }
    curs_set(0);
    noecho();
    cbreak();

    start_color();
    init_color(COLOR_BLACK, 0, 0, 0);
    init_color(COLOR_GREEN, 0, 600, 0);
    init_color(COLOR_RED, 1000, 0, 150);
    init_pair((uint8_t)SystemMonitorProcess::Color::NO_COLOR, COLOR_WHITE, COLOR_BLACK);
    init_pair((uint8_t)SystemMonitorProcess::Color::RED_COLOR, COLOR_WHITE, COLOR_RED);
    init_pair((uint8_t)SystemMonitorProcess::Color::YELLOW_COLOR, COLOR_WHITE, COLOR_YELLOW);
    init_pair((uint8_t)SystemMonitorProcess::Color::GREEN_COLOR, COLOR_WHITE, COLOR_GREEN);
    init_pair((uint8_t)SystemMonitorProcess::Color::BLUE_COLOR, COLOR_WHITE, COLOR_BLUE);

    uint16_t mainwindow_width, mainwindow_height;
    getmaxyx(stdscr, mainwindow_height, mainwindow_width);
    bool status = process->set_mainwindow(mainwindow_width, mainwindow_height);
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Window: Width: " + std::to_string(mainwindow_width) + " Height: " +
                          std::to_string(mainwindow_height) + " is too small. Exiting.");
        return false;
    }
    status = process->initialize_windows();
    if (status == false) {
        logger->enable_consoleprint();
        logger->log_error("Unable to initialize Windows. Exiting. ");
    }
    return true;
}
void SystemMonitorNode::heartbeat_Callback(const eros::heartbeat::ConstPtr &t_msg) {
    Diagnostic::DiagnosticDefinition diag = process->new_heartbeatmessage(t_msg);
    if (diag.level > Level::Type::NOTICE) {
        logger->log_diagnostic(diag);
    }
}
Diagnostic::DiagnosticDefinition SystemMonitorNode::rescan_nodes() {
    Diagnostic::DiagnosticDefinition diag = process->get_root_diagnostic();
    std::size_t found_new_subscribers = 0;
    ros::V_string nodes;
    ros::master::getNodes(nodes);
    std::vector<std::string> node_list;
    for (ros::V_string::iterator it = nodes.begin(); it != nodes.end(); it++) {
        const std::string &node_name = *it;
        std::size_t found = node_name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }
        node_list.push_back(node_name);
    }
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::vector<std::string> heartbeat_list;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end();
         it++) {
        const ros::master::TopicInfo &info = *it;
        std::size_t found = info.name.find(BASE_NODE_NAME);
        if (found != std::string::npos) {
            continue;
        }

        if (info.datatype == "eros/heartbeat") {
            heartbeat_list.push_back(info.name);
        }
    }
    std::vector<std::string> new_heartbeat_topics_to_subscribe;
    diag = process->update_nodelist(node_list, heartbeat_list, new_heartbeat_topics_to_subscribe);
    if (diag.level >= Level::Type::WARN) {
        logger->log_diagnostic(diag);
        return diag;
    }
    for (std::size_t i = 0; i < new_heartbeat_topics_to_subscribe.size(); ++i) {
        ros::Subscriber sub = n->subscribe<eros::heartbeat>(new_heartbeat_topics_to_subscribe.at(i),
                                                            2,
                                                            &SystemMonitorNode::heartbeat_Callback,
                                                            this);
        heartbeat_subs.push_back(sub);
    }
    found_new_subscribers = new_heartbeat_topics_to_subscribe.size();
    if (found_new_subscribers > 0) {
        logger->log_info("Rescanned and found " + std::to_string(found_new_subscribers) +
                         " new things to subscribe to.");
    }
    else {
        logger->log_info("Rescanned and found no new things to subscribe to.");
    }
    return diag;
}
void signalinterrupt_handler(int sig) {
    printf("Killing SystemMonitorNode with Signal: %d", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    SystemMonitorNode *node = new SystemMonitorNode();
    bool status = node->start(argc, argv);
    std::thread thread(&SystemMonitorNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}