#include "DeviceWindow/DeviceWindow.h"
namespace eros_nodes::SystemMonitor {
DeviceWindow::~DeviceWindow() {
}
eros::Diagnostic::DiagnosticDefinition DeviceWindow::update(double dt, double t_ros_time) {
    eros::Diagnostic::DiagnosticDefinition diag = BaseWindow::update(dt, t_ros_time);
    box(get_window(), 0, 0);
    wrefresh(get_window());
    return diag;
}
}  // namespace eros_nodes::SystemMonitor