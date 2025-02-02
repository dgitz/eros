#include "DiagnosticsWindow/DiagnosticsWindow.h"
namespace eros_nodes::SystemMonitor {
DiagnosticsWindow::~DiagnosticsWindow() {
}

bool DiagnosticsWindow::update(double dt, double t_ros_time) {
    bool status = BaseWindow::update(dt, t_ros_time);
    if (status == false) {
        return false;
    }

    logger->log_warn("Update");

    request_data_timer += dt;
    if (request_data_timer >= REQUEST_DATA_RATE) {
        request_data_timer = 0.0;
        std::string system_diagnostic_topic = robot_namespace + "srv_system_diagnostics";
        ros::ServiceClient system_diag_client =
            nodeHandle->serviceClient<eros::srv_get_diagnostics>(system_diagnostic_topic);
        eros::srv_get_diagnostics srv;
        srv.request.MinLevel = 0;
        srv.request.DiagnosticType = 0;
        if (system_diag_client.call(srv)) {
            for (auto eros_diag : srv.response.diag_list) {
                eros::Diagnostic::DiagnosticDefinition diag = eros::convert(eros_diag);
                system_diagnostics[(uint8_t)diag.type].diagnostic = diag;
            }
        }
        else {
            logger->log_warn("Unable to request System Diagnostics.");
        }
    }
    status = update_window();
    return status;
}
bool DiagnosticsWindow::update_window() {
    uint16_t index = 0;
    for (auto& diag : system_diagnostics) {
        mvwprintw(get_window(), 1, 1, "System Diagnostics:");
        wclrtoeol(get_window());
        std::string dashed(get_screen_coordinates_pixel().width_pix - 2, '-');
        mvwprintw(get_window(), 2, 1, dashed.c_str());
        eros_nodes::SystemMonitor::Color color;
        std::string str =
            "  " + eros::Diagnostic::DiagnosticTypeString(diag.second.diagnostic.type);
        if (diag.second.diagnostic.message == eros::Diagnostic::Message::NODATA) {
            color = Color::NO_COLOR;
        }
        else {
            switch (diag.second.diagnostic.level) {
                case eros::Level::Type::DEBUG: color = Color::NO_COLOR; break;
                case eros::Level::Type::INFO: color = Color::GREEN_COLOR; break;
                case eros::Level::Type::NOTICE: color = Color::GREEN_COLOR; break;
                case eros::Level::Type::WARN:
                    color = eros_nodes::SystemMonitor::Color::YELLOW_COLOR;
                    str += ": " + diag.second.diagnostic.description;
                    break;
                case eros::Level::Type::ERROR:
                    color = eros_nodes::SystemMonitor::Color::RED_COLOR;
                    str += ": " + diag.second.diagnostic.description;
                    break;
                case eros::Level::Type::FATAL:
                    color = eros_nodes::SystemMonitor::Color::RED_COLOR;
                    str += ": " + diag.second.diagnostic.description;
                    break;
                default: color = Color::RED_COLOR; break;
            }
        }

        str += "  ";
        if (str.size() > (std::size_t)(get_screen_coordinates_pixel().width_pix - 4)) {
            str =
                str.substr(0, (std::size_t)(get_screen_coordinates_pixel().width_pix - 4)) + "...";
        }
        wattron(get_window(), COLOR_PAIR(color));
        mvwprintw(get_window(), index + 3, 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
        index++;
    }
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