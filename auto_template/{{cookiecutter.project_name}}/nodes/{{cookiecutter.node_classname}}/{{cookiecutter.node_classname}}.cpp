#include "{{cookiecutter.node_classname}}.h"
bool kill_node = false;
{{cookiecutter.node_classname}}::{{cookiecutter.node_classname}}() {
}
{{cookiecutter.node_classname}}::~{{cookiecutter.node_classname}}() {
}
bool {{cookiecutter.node_classname}}::changenodestate_service(eros::srv_change_nodestate::Request &req,
                             eros::srv_change_nodestate::Response &res)
{
    Node::State req_state = Node::NodeState(req.RequestedNodeState);
    process->request_statechange(req_state);
    res.NodeState = Node::NodeStateString(process->get_nodestate());
    return true;
}
bool {{cookiecutter.node_classname}}::start(int argc, char **argv) {
    initialize_diagnostic(DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
    bool status = false;
    process = new {{cookiecutter.process_classname}}();
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
Diagnostic::DiagnosticDefinition {{cookiecutter.node_classname}}::read_launchparameters() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    get_logger()->log_notice("Configuration Files Loaded.");
    return diag;
}
Diagnostic::DiagnosticDefinition {{cookiecutter.node_classname}}::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    std::string srv_nodestate_topic = "/" + node_name + "/srv_nodestate_change";
    nodestate_srv = n->advertiseService(srv_nodestate_topic, &{{cookiecutter.node_classname}}::changenodestate_service, this);
    return diag;
}
bool {{cookiecutter.node_classname}}::run_loop1() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_loop2() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_loop3() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_001hz() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_01hz() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_01hz_noisy() {
    logger->log_notice("Node State: " + Node::NodeStateString(process->get_nodestate()));
    return true;
}
bool {{cookiecutter.node_classname}}::run_1hz() {
    return true;
}
bool {{cookiecutter.node_classname}}::run_10hz() {
    update_diagnostics(process->get_diagnostics());
    return true;
}
void {{cookiecutter.node_classname}}::thread_loop() {
    while (kill_node == false) { ros::Duration(1.0).sleep(); }
}
void {{cookiecutter.node_classname}}::cleanup() {
    process->request_statechange(Node::State::FINISHED);
    process->cleanup();
    delete process;
    base_cleanup();
}
void signalinterrupt_handler(int sig) {
    printf("Killing {{cookiecutter.node_classname}} with Signal: %d\n", sig);
    kill_node = true;
    exit(0);
}
int main(int argc, char **argv) {
    signal(SIGINT, signalinterrupt_handler);
    signal(SIGTERM, signalinterrupt_handler);
    {{cookiecutter.node_classname}} *node = new {{cookiecutter.node_classname}}();
    bool status = node->start(argc, argv);
    if (status == false) {
        return EXIT_FAILURE;
    }
    std::thread thread(&{{cookiecutter.node_classname}}::thread_loop, node);
    while ((status == true) and (kill_node == false)) {
        status = node->update(node->get_process()->get_nodestate());
    }
    node->cleanup();
    thread.detach();
    delete node;
    return 0;
}
