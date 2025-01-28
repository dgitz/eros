#include "DeviceWindow/DeviceWindow.h"
namespace eros_nodes::SystemMonitor {
DeviceWindow::~DeviceWindow() {
}
eros::Diagnostic::DiagnosticDefinition DeviceWindow::update(double dt, double t_ros_time) {
    eros::Diagnostic::DiagnosticDefinition diag = BaseWindow::update(dt, t_ros_time);
    std::map<std::string, Device>::iterator device_it = device_list.begin();
    while (device_it != device_list.end()) {
        device_it->second.last_heartbeat_delta += t_dt;
        if (device_it->second.last_heartbeat_delta >= 4.0 * COMMTIMEOUT_THRESHOLD) {
            device_it->second.state = Node::State::UNKNOWN;
        }
        ++device_it;
    }
    std::map<std::string, Device>::iterator device_it;
    const uint16_t DEVICESTART_COORD_Y = 1;
    const uint16_t DEVICESTART_COORD_X = 1;
    for (uint16_t i = 0; i < (uint16_t)device_list.size(); ++i) {
        Color color = Color::UNKNOWN;
        switch (device_it->second.state) {
            case Node::State::UNKNOWN: color = Color::RED_COLOR; break;
            case Node::State::START: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZING: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZED: color = Color::YELLOW_COLOR; break;
            case Node::State::RUNNING: color = Color::BLUE_COLOR; break;
            case Node::State::PAUSED: color = Color::GREEN_COLOR; break;
            case Node::State::RESET: color = Color::YELLOW_COLOR; break;
            case Node::State::FINISHED: color = Color::YELLOW_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }
        std::string str = get_device_info(device_it->second, false);  /// ADD THIS!!!
        wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
        mvwprintw(window_it->second.get_window_reference(),
                  DEVICESTART_COORD_Y + 2 + (int)index,
                  DEVICESTART_COORD_X + 1,
                  str.c_str());
        wclrtoeol(window_it->second.get_window_reference());
        wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        devices_shown++;
        index++;
    }
    box(window_it->second.get_window_reference(), 0, 0);
    wrefresh(window_it->second.get_window_reference());
    return diag;
    return diag;
}
bool DeviceWindow::new_msg(eros::resource resource_available_msg) {
    device_list_mutex.lock();
    std::map<std::string, DeviceData>::iterator device_it;

    device_it = device_list.find(resource_available_msg.Name);
    if (device_it != device_list.end()) {  // Found it, update the record
        device_it->second.last_heartbeat_delta = 0.0;
    }
    else {  // Didn't find it,create one
        device_list_mutex.unlock();
        return insertDevice(resource_available_msg.Name);
    }
    device_list_mutex.unlock();
    return true;
}
bool DeviceWindow::insertDevice(std::string device_name) {
    std::lock_guard<std::mutex> guard(device_list_mutex);
    std::size_t before = device_list.size();
    DeviceData newDevice(device_list.size(), device_name);
    std::string key = newDevice.name;
    device_list.insert(std::pair<std::string, DeviceData>(key, newDevice));
    std::size_t after = device_list.size();
    return after > before;
}
}  // namespace eros_nodes::SystemMonitor