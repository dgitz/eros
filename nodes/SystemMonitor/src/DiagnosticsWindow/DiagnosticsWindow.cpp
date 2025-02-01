#include "DiagnosticsWindow/DiagnosticsWindow.h"
namespace eros_nodes::SystemMonitor {
DiagnosticsWindow::~DiagnosticsWindow() {
}

bool DiagnosticsWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }
    std::string system_diagnostic_topic = robot_namespace + "srv_system_diagnostics";
    ros::ServiceClient system_diag_client =
        nodeHandle->serviceClient<eros::srv_get_diagnostics>(system_diagnostic_topic);
    eros::srv_get_diagnostics srv;
    srv.request.MinLevel = 0;
    srv.request.DiagnosticType = 0;
    if (system_diag_client.call(srv)) {
        for (auto diag : srv.response.diag_list) { logger->log_debug("Diag: " + diag.Description); }
    }
    else {
        logger->log_warn("Unable to request System Diagnostics.");
    }
    status = update_window();
    return status;
}
bool DiagnosticsWindow::update_window() {
    if (focused) {
        box(get_window(), '.', '.');
    }
    else {
        box(get_window(), 0, 0);
    }
    wrefresh(get_window());
    return true;
}
}  // namespace eros_nodes::SystemMonitor