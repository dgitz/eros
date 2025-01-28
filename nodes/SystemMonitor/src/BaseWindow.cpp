#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
eros::Diagnostic::DiagnosticDefinition BaseWindow::update(double dt, double t_ros_time) {
    eros::Diagnostic::DiagnosticDefinition diag = root_diagnostic;
    t_ros_time_ = t_ros_time;
    return diag;
}
ScreenCoordinatePixel BaseWindow::convertCoordinate(ScreenCoordinatePerc coord_perc,
                                                    uint16_t width_pix,
                                                    uint16_t height_pix) {
    ScreenCoordinatePixel coord(0, 0, 0, 0);
    coord.start_x_pix = (uint16_t)((double)width_pix * (.01 * coord_perc.start_x_perc));
    coord.start_y_pix = (uint16_t)((double)height_pix * (.01 * coord_perc.start_y_perc));
    coord.width_pix = (uint16_t)((double)width_pix * (.01 * coord_perc.width_perc));
    coord.height_pix = (uint16_t)((double)height_pix * (.01 * coord_perc.height_perc));
    return coord;
}
}  // namespace eros_nodes::SystemMonitor