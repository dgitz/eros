#include <eros/ResourceMonitor.h>
ResourceMonitor::ResourceMonitor() {
}
ResourceMonitor::ResourceMonitor(Architecture::Type _architecture,
                                 Mode _mode,
                                 Diagnostic::DiagnosticDefinition _diag)
    : architecture(_architecture),
      mode(_mode),
      diagnostic(_diag),
      initialized(false),
      run_time(0.0) {
    resourceInfo.cpu_perc = -1.0;
    resourceInfo.disk_perc = -1.0;
    resourceInfo.ram_mb = -1.0;
    diagnostic.type = Diagnostic::DiagnosticType::SYSTEM_RESOURCE;
    diagnostic.update_count = 0;
}
ResourceMonitor::~ResourceMonitor() {
}
Diagnostic::DiagnosticDefinition ResourceMonitor::init() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    if (architecture == Architecture::Type::UNKNOWN) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
        initialized = false;
    }
    if ((architecture != Architecture::Type::X86_64)) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description =
            "Architecture: " + Architecture::ArchitectureString(architecture) + " Not Supported.";
        diag.update_count++;
        initialized = false;
    }
    resourceInfo.pid = ::getpid();
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
    }
    if (diag.level <= Level::Type::NOTICE) {
        initialized = true;
    }
    else {
        initialized = false;
    }
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::update(double t_dt) {
    Diagnostic::DiagnosticDefinition diag = diagnostic;
    run_time += t_dt;
    if (architecture == Architecture::Type::UNKNOWN) {
        diag.level = Level::Type::ERROR;
        diag.message = Diagnostic::Message::INITIALIZING_ERROR;
        diag.description = "Architecture Not Supported.";
        diag.update_count++;
    }
    if (mode == Mode::PROCESS) {
        diag = read_process_resource_usage();
    }
    else if (mode == Mode::DEVICE) {
        diag = read_device_resource_availability();
    }
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::read_process_resource_usage() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;

    diag.level = Level::Type::ERROR;
    diag.message = Diagnostic::Message::INITIALIZING_ERROR;
    diag.description = "Not Implemented Yet.";
    diag.update_count++;
    return diag;
}
Diagnostic::DiagnosticDefinition ResourceMonitor::read_device_resource_availability() {
    Diagnostic::DiagnosticDefinition diag = diagnostic;

    diag.level = Level::Type::ERROR;
    diag.message = Diagnostic::Message::INITIALIZING_ERROR;
    diag.description = "Not Implemented Yet.";
    diag.update_count++;
    return diag;
}
std::string ResourceMonitor::pretty(ResourceInfo info) {
    std::string str = "--- Resource Monitor Info ---\n";
    str += "\tPID: " + std::to_string(info.pid);
    return str;
}