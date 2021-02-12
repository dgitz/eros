#include <test_node/SampleNode.h>
bool kill_node = false;
SampleNode::SampleNode() {
}
SampleNode::~SampleNode() {
}
bool SampleNode::start(int argc, char **argv) {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new SampleNodeProcess();
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
Diagnostic::DiagnosticDefinition SampleNode::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition SampleNode::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    return diag;
}
bool SampleNode::run_loop1() {
    logger->log_debug("Loop1");
    return true;
}
bool SampleNode::run_loop2() {
    logger->log_debug("Loop2");
    return true;
}
bool SampleNode::run_loop3() {
    logger->log_debug("Loop3");
    return true;
}
bool SampleNode::run_001hz() {
    logger->log_debug("Loop .001 Hz");
    return true;
}
bool SampleNode::run_01hz() {
    logger->log_debug("Loop .01 Hz");

    return true;
}
bool SampleNode::run_01hz_noisy() {
    logger->log_debug("Loop .01 Hz (Noisy)");
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool SampleNode::run_1hz() {
    logger->log_debug("Loop 1 Hz");
    return true;
}
bool SampleNode::run_10hz() {
    logger->log_debug("Loop 10 Hz");
    return true;
}
void SampleNode::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void SampleNode::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    delete process;
}
void signalinterrupt_handler(int sig) {
    printf("Killing SampleNode with Signal: %d", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    SampleNode *node = new SampleNode();
    bool status = node->start(argc, argv);
    std::thread thread(&SampleNode::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    return 0;
}