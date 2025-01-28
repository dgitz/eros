#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
eros::Diagnostic::DiagnosticDefinition BaseWindow::update(double /* dt */, double t_ros_time) {
    eros::Diagnostic::DiagnosticDefinition diag = root_diagnostic;
    t_ros_time_ = t_ros_time;
    return diag;
}

}  // namespace eros_nodes::SystemMonitor