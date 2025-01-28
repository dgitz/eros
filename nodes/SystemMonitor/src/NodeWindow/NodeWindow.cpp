#include "NodeWindow.h"
namespace eros_nodes::SystemMonitor {
NodeWindow::~NodeWindow() {
}
bool NodeWindow::new_msg(eros::ArmDisarm::State armed_state) {
    return true;
}
eros::Diagnostic::DiagnosticDefinition NodeWindow::update(double dt, double t_ros_time) {
    eros::Diagnostic::DiagnosticDefinition diag = root_diagnostic;
    t_ros_time_ = t_ros_time;
    std::map<std::string, NodeData>::iterator node_it = node_list.begin();
    while (node_it != node_list.end()) {
        node_it->second.last_heartbeat_delta += dt;
        if (node_it->second.last_heartbeat_delta >= COMMTIMEOUT_THRESHOLD) {
            node_it->second.state = eros::Node::State::UNKNOWN;
        }
        ++node_it;
    }
    const uint16_t TASKSTART_COORD_Y = 1;
    const uint16_t TASKSTART_COORD_X = 1;
    uint16_t index = 0;
    for (node_it = node_list.begin(); node_it != node_list.end(); node_it++) {
        Color color = Color::UNKNOWN;
        switch (node_it->second.state) {
            case eros::Node::State::UNKNOWN: color = Color::RED_COLOR; break;
            case eros::Node::State::START: color = Color::YELLOW_COLOR; break;
            case eros::Node::State::INITIALIZING: color = Color::YELLOW_COLOR; break;
            case eros::Node::State::INITIALIZED: color = Color::YELLOW_COLOR; break;
            case eros::Node::State::RUNNING: color = Color::BLUE_COLOR; break;
            case eros::Node::State::PAUSED: color = Color::GREEN_COLOR; break;
            case eros::Node::State::RESET: color = Color::YELLOW_COLOR; break;
            case eros::Node::State::FINISHED: color = Color::YELLOW_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }

        wattron(get_window(), COLOR_PAIR(color));
        std::string str = get_node_info(node_it->second, false);
        mvwprintw(
            get_window(), TASKSTART_COORD_Y + 2 + (int)index, TASKSTART_COORD_X + 1, str.c_str());
        wclrtoeol(get_window());
        wattroff(get_window(), COLOR_PAIR(color));
        index++;
    }

    box(get_window(), 0, 0);
    wrefresh(get_window());
    return diag;
}
bool NodeWindow::new_msg(eros::heartbeat heartbeat_msg) {
    std::map<std::string, NodeData>::iterator node_it;

    node_it = node_list.find(heartbeat_msg.NodeName);
    if (node_it != node_list.end()) {  // Found it, update the record
        node_it->second.last_heartbeat_delta = 0.0;
        node_it->second.last_heartbeat = t_ros_time_;
    }
    else {  // Didn't find it,create one
        NodeData newNode(node_list.size(),
                         NodeType::EROS,
                         "Unknown",
                         heartbeat_msg.BaseNodeName,
                         heartbeat_msg.NodeName);
        newNode.state = eros::Node::State::RUNNING;
        std::string key = newNode.node_name;
        node_list.insert(std::pair<std::string, NodeData>(key, newNode));
    }
    return true;
}
std::string NodeWindow::get_node_info(NodeData node, bool selected) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = node_window_fields.find(NodeFieldColumn::MARKER)->second.width;
        for (std::size_t i = 0; i < width; ++i) {
            if (selected == true) {
                str += "*";
            }
            else {
                str += " ";
            }
        }
    }
    {
        width = node_window_fields.find(NodeFieldColumn::ID)->second.width;
        std::string tempstr = std::to_string(node.id);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::HOSTNAME)->second.width;
        std::string tempstr = node.host_device;
        if (tempstr.size() > width) {
            tempstr = tempstr.substr(0, width - 4) + "... ";
        }
        else {
            if (tempstr.size() > (std::size_t)(width - 1)) {
                tempstr = tempstr.substr(0, (width - 1));
                tempstr += " ";
            }
            else {
                std::size_t spaces = width - tempstr.size();
                if (spaces > 0) {
                    tempstr += std::string(spaces, ' ');
                }
            }
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::NODENAME)->second.width;
        std::string tempstr = node.node_name;
        std::size_t found_hostname = node.node_name.find(node.host_device);
        if (found_hostname != std::string::npos) {
            tempstr.replace(found_hostname, node.host_device.length(), "");
        }
        if (tempstr.size() > width) {
            tempstr = tempstr.substr(0, width - 4) + "... ";
        }
        else {
            std::size_t spaces = width - tempstr.size();
            if (spaces > 0) {
                tempstr += std::string(spaces, ' ');
            }
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::STATUS)->second.width;
        std::string tempstr = eros::Node::NodeStateString(node.state);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RESTARTS)->second.width;
        std::string tempstr = std::to_string(node.restart_count);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::PID)->second.width;
        std::string tempstr = std::to_string(node.pid);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::CPU)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", node.cpu_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RAM)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", node.mem_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = node_window_fields.find(NodeFieldColumn::RX)->second.width;
        std::string max_number_str(width - 4, '9');
        double max_num = std::atof(max_number_str.c_str()) + 0.99;
        if (node.last_heartbeat_delta > max_num) {
            node.last_heartbeat_delta = max_num;
        }
        char tempstr[2 * width];
        sprintf(tempstr, "%2.2f", node.last_heartbeat_delta);
        std::string tempstr_str = std::string(tempstr);
        std::size_t spaces = width - tempstr_str.size();
        if (spaces > 0) {
            tempstr_str += std::string(spaces, ' ');
        }
        str += tempstr_str;
    }

    return str;
}
}  // namespace eros_nodes::SystemMonitor