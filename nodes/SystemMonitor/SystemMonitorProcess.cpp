#include <eros/SystemMonitor/SystemMonitorProcess.h>

WINDOW *create_newwin(int height, int width, int starty, int startx) {
    WINDOW *local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0); /* 0, 0 gives default characters
                           * for the vertical and horizontal
                           * lines			*/
    wrefresh(local_win);  /* Show that box 		*/

    return local_win;
}
using namespace eros;
using namespace eros_nodes;
SystemMonitorProcess::~SystemMonitorProcess() {
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::finish_initialization() {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Finished Initialization.");
    return diag;
}
void SystemMonitorProcess::reset() {
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update(double t_dt, double t_ros_time) {
    Diagnostic::DiagnosticDefinition diag = base_update(t_dt, t_ros_time);
    timer_showing_message_in_window += t_dt;

    if (timer_showing_message_in_window > TIME_TO_SHOW_MESSAGES) {
        message_text = "";
        message_text_color = Color::NO_COLOR;
    }
    std::map<std::string, Task>::iterator task_it = task_list.begin();
    while (task_it != task_list.end()) {
        task_it->second.last_heartbeat_delta += t_dt;
        if (task_it->second.last_heartbeat_delta >= COMMTIMEOUT_THRESHOLD) {
            task_it->second.state = Node::State::NODATA;
        }
        ++task_it;
    }
    std::map<std::string, Device>::iterator device_it = device_list.begin();
    while (device_it != device_list.end()) {
        device_it->second.last_heartbeat_delta += t_dt;
        if (device_it->second.last_heartbeat_delta >= 4.0 * COMMTIMEOUT_THRESHOLD) {
            device_it->second.state = Node::State::NODATA;
        }
        ++device_it;
    }

    std::map<std::string, WindowManager>::iterator win_it = windows.begin();
    while (win_it != windows.end()) {
        if (win_it->first == "header") {
            diag = update_headerwindow(win_it);
            if (diag.level > Level::Type::ERROR) {
                return diag;
            }
        }
        else if (win_it->first == "instruction_window") {
            diag = update_instructionwindow(win_it);
            if (diag.level > Level::Type::ERROR) {
                return diag;
            }
        }

        else if (win_it->first == "task_window") {
            diag = update_taskwindow(win_it);
            if (diag.level > Level::Type::WARN) {
                return diag;
            }
        }
        else if (win_it->first == "message_window") {
            diag = update_messagewindow(win_it);
            if (diag.level > Level::Type::WARN) {
                return diag;
            }
        }
        else if (win_it->first == "diag_sidebar") {
            diag = update_diagnosticwindow(win_it);
            if (diag.level > Level::Type::WARN) {
                return diag;
            }
        }
        else if (win_it->first == "device_window") {
            diag = update_devicewindow(win_it);
            if (diag.level > Level::Type::WARN) {
                return diag;
            }
        }
        else {
            logger->log_debug("Window: " + win_it->first + " Not Supported.");
        }
        if (DEBUG_MODE == true) {
            mvwprintw(win_it->second.get_window_reference(), 1, 1, win_it->second.pretty().c_str());
        }
        wrefresh(win_it->second.get_window_reference());
        ++win_it;
    }
    flushinp();

    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_nodelist(
    std::vector<std::string> node_list,
    std::vector<std::string> heartbeat_list,
    std::vector<std::string> &new_heartbeat_topics_to_subscribe,
    std::vector<std::string> &new_resourceused_topics_to_subscribe) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    // Iterate over Node List.  If NEW, add as non EROS.  Do nothing for heart beat sub list
    {
        std::vector<std::string>::iterator it = node_list.begin();
        while (it != node_list.end()) {
            std::map<std::string, Task>::iterator task_it;
            task_it = task_list.find(*it);
            if (task_it == task_list.end()) {
                Task newTask(task_list.size(), TaskType::NON_EROS, "Unknown", "Unknown", *it);
                newTask.state = Node::State::RUNNING;
                std::string key = newTask.node_name;
                task_list.insert(std::pair<std::string, Task>(key, newTask));
                task_name_list.insert(std::pair<uint16_t, std::string>(newTask.id, key));
            }
            else {
                task_it->second.last_heartbeat_delta = 0.0;
                task_it->second.last_heartbeat = get_system_time();
            }
            ++it;
        }
    }
    // Iterate over Heartbeat List.  Determine Node Name from topic name.
    // If Node does not exist, add as EROS (append sub).  If Node does exist but is NON-EROS,
    // change to EROS. (append sub)
    {
        std::vector<std::string>::iterator it = heartbeat_list.begin();
        while (it != heartbeat_list.end()) {
            std::string key = *it;
            std::size_t found = key.substr(1, key.size()).find_last_of("/");
            if (found != std::string::npos) {
                key = key.substr(0, found + 1);
            }
            std::map<std::string, Task>::iterator task_it = task_list.find(key);
            if (task_it == task_list.end()) {
                Task newTask(task_list.size(), TaskType::EROS, "Unknown", "Unknown", key);
                std::string key = newTask.node_name;
                task_list.insert(std::pair<std::string, Task>(key, newTask));
                task_name_list.insert(std::pair<uint16_t, std::string>(newTask.id, key));
                new_heartbeat_topics_to_subscribe.push_back(*it);
                new_resourceused_topics_to_subscribe.push_back(newTask.node_name +
                                                               "/resource_used");
            }
            else if (task_it->second.type == TaskType::NON_EROS) {
                task_it->second.type = TaskType::EROS;
                new_heartbeat_topics_to_subscribe.push_back(*it);
                new_resourceused_topics_to_subscribe.push_back(task_it->second.node_name +
                                                               "/resource_used");
            }
            ++it;
        }
    }

    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Processed Update.");
    return diag;
}

Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_devicelist(
    std::vector<std::string> loadfactor_list,
    std::vector<std::string> &new_resourceavailable_topics_to_subscribe,
    std::vector<std::string> &new_loadfactor_topics_to_subscribe) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    std::map<std::string, Device>::iterator device_it;
    std::map<uint16_t, std::string>::iterator device_name_it;
    std::vector<std::string>::iterator it = loadfactor_list.begin();
    while (it != loadfactor_list.end()) {
        std::string key = *it;
        key = key.substr(0, key.find("loadfactor") - 1);
        device_it = device_list.find(key);

        if (device_it == device_list.end()) {
            Device newDevice(device_list.size(), key);
            device_list.insert(std::make_pair(newDevice.name, newDevice));
            device_name_list.insert(std::make_pair(newDevice.id, newDevice.name));
            new_resourceavailable_topics_to_subscribe.push_back(key + "/resource_available");
            new_loadfactor_topics_to_subscribe.push_back(key + "/loadfactor");
        }
        ++it;
    }
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Updated Device List.");
    return diag;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::new_commandmsg(
    eros::command msg) {
    (void)msg;  // Not currently used.
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Processed Command.");
    diag_list.push_back(diag);
    return diag_list;
}
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::check_programvariables() {
    std::vector<Diagnostic::DiagnosticDefinition> diag_list;
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                               Level::Type::INFO,
                                               Diagnostic::Message::NOERROR,
                                               "Program Variable Check Passed.");
    diag_list.push_back(diag);
    return diag_list;
}
WindowManager::ScreenCoordinatePixel SystemMonitorProcess::convertCoordinate(
    WindowManager::ScreenCoordinatePerc coord_perc, uint16_t width_pix, uint16_t height_pix) {
    WindowManager::ScreenCoordinatePixel coord(0, 0, 0, 0);
    coord.start_x_pix = (uint16_t)((double)width_pix * (.01 * coord_perc.start_x_perc));
    coord.start_y_pix = (uint16_t)((double)height_pix * (.01 * coord_perc.start_y_perc));
    coord.width_pix = (uint16_t)((double)width_pix * (.01 * coord_perc.width_perc));
    coord.height_pix = (uint16_t)((double)height_pix * (.01 * coord_perc.height_perc));
    return coord;
}
bool SystemMonitorProcess::initialize_windows() {
    // Initialize Windows
    {
        WindowManager window("header", 0.0, 0.0, 100.0, 15.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }
    {
        WindowManager window("task_window", 0.0, 15.0, 66.0, 60.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    {
        WindowManager window("diag_sidebar", 66, 15.0, 34.5, 60.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    {
        WindowManager window("message_window", 0, 75.0, 100.0, 7.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    {
        WindowManager window("status_window", 0, 80.0, 30.0, 20.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    {
        WindowManager window("instruction_window", 30, 80.0, 25.0, 20.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }
    {
        WindowManager window("device_window", 55.0, 80.0, 45.0, 20.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    std::map<std::string, WindowManager>::iterator it = windows.begin();
    while (it != windows.end()) {
        WindowManager::ScreenCoordinatePixel coord_pix = convertCoordinate(
            it->second.get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW *win = create_newwin(coord_pix.height_pix,
                                    coord_pix.width_pix,
                                    coord_pix.start_y_pix,
                                    coord_pix.start_x_pix);
        it->second.set_screen_coordinates_pix(coord_pix);
        it->second.set_window_reference(win);

        if (it->first == "header") {
            wbkgd(it->second.get_window_reference(), COLOR_PAIR(Color::NO_COLOR));
        }
        else if (it->first == "instruction_window") {
            std::string str = "Instructions:";
            keypad(it->second.get_window_reference(), TRUE);
            mvwprintw(it->second.get_window_reference(), 1, 1, str.c_str());
            std::string dashed(coord_pix.width_pix - 2, '-');
            mvwprintw(it->second.get_window_reference(), 2, 1, dashed.c_str());
            wtimeout(it->second.get_window_reference(), 0);
        }
        else if (it->first == "task_window") {
            task_list_max_rows = coord_pix.height_pix - 5;
            if ((coord_pix.height_pix - 5) < 0) {
                logger->log_warn("Screen Too Small. Closing.");
                return false;
            }
            keypad(it->second.get_window_reference(), TRUE);
            std::string header = get_taskheader();
            mvwprintw(it->second.get_window_reference(), 1, 1, header.c_str());
            std::string dashed(it->second.get_screen_coordinates_pixel().width_pix - 2, '-');
            mvwprintw(it->second.get_window_reference(), 2, 1, dashed.c_str());
            wtimeout(it->second.get_window_reference(), 0);
        }
        else if (it->first == "diag_sidebar") {
        }
        else if (it->first == "device_window") {
            std::string header = get_deviceheader();
            mvwprintw(it->second.get_window_reference(), 1, 1, header.c_str());
            std::string dashed(it->second.get_screen_coordinates_pixel().width_pix - 2, '-');
            mvwprintw(it->second.get_window_reference(), 2, 1, dashed.c_str());
        }
        else {
            logger->log_debug("Window: " + it->first + " Not Supported.");
        }
        wrefresh(it->second.get_window_reference());
        ++it;
    }
    return true;
}
std::string SystemMonitorProcess::get_taskheader() {
    std::string str = "";
    std::map<TaskFieldColumn, Field>::iterator it = task_window_fields.begin();
    while (it != task_window_fields.end()) {
        // Check if field name is too long:
        if (it->second.text.size() > it->second.width) {
            str += it->second.text.substr(0, it->second.width);
        }
        else {
            str += it->second.text;
            // Figure out how many spaces to add
            std::size_t spaces = it->second.width - it->second.text.size();
            for (std::size_t j = 0; j < spaces; ++j) { str += " "; }
        }
        ++it;
    }

    if (str.size() > mainwindow_width) {
        logger->enable_consoleprint();
        logger->log_warn("Task Header too long for Window!.");
        return "";
    }
    return str;
}
std::string SystemMonitorProcess::get_deviceheader() {
    std::string str = "";
    std::map<DeviceFieldColumn, Field>::iterator it = device_window_fields.begin();
    while (it != device_window_fields.end()) {
        // Check if field name is too long:
        if (it->second.text.size() > it->second.width) {
            str += it->second.text.substr(0, it->second.width);
        }
        else {
            str += it->second.text;
            // Figure out how many spaces to add
            std::size_t spaces = it->second.width - it->second.text.size();
            for (std::size_t j = 0; j < spaces; ++j) { str += " "; }
        }
        ++it;
    }

    if (str.size() > mainwindow_width) {
        logger->enable_consoleprint();
        logger->log_warn("Device Header too long for Window!.");
        return "";
    }
    return str;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_headerwindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    {  // Armed State
        Color color;
        std::string str = "Armed State: " + ArmDisarm::ArmDisarmString(armed_state.state);
        str.insert(str.end(), 40 - str.size(), ' ');
        switch (armed_state.state) {
            case ArmDisarm::Type::ARMED:
                color = Color::GREEN_COLOR;
                break;  // Should be BLUE for RC Mode, GREEN for Manual, PURPLE for Auto
            case ArmDisarm::Type::DISARMED_CANNOTARM: color = Color::RED_COLOR; break;
            case ArmDisarm::Type::DISARMED: color = Color::GREEN_COLOR; break;
            case ArmDisarm::Type::DISARMING: color = Color::GREEN_COLOR; break;
            case ArmDisarm::Type::ARMING: color = Color::GREEN_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }
        wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
        mvwprintw(window_it->second.get_window_reference(), 2, 1, str.c_str());
        // wclrtoeol(window_it->second.get_window_reference());
        wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        return diag;
    }
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_instructionwindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();

    if (select_task_mode == true) {
        std::vector<std::string> instruction_string;
        instruction_string.push_back("S: Generate System Snapshot. (C: Clear Snapshots)");
        instruction_string.push_back("F: Get Node Firmware.");
        instruction_string.push_back("L: Change Log Level.");
        if (change_log_level_mode == true) {
            instruction_string.push_back("  1,2,3,4,5,6: Select Log Level.");
        }
        if (show_task_diagnostic_mode == false) {
            instruction_string.push_back("D: Show Task Diagnostics.");
        }
        else {
            instruction_string.push_back("D: Show System Diagnostics.");
        }
        instruction_string.push_back("N: Change Node State.");
        if (change_nodestate_mode == true) {
            instruction_string.push_back("  1-9: Select Node State.");
        }
        for (std::size_t i = 0; i < instruction_string.size(); ++i) {
            mvwprintw(window_it->second.get_window_reference(),
                      i + 3,
                      1,
                      instruction_string.at(i).c_str());
            wclrtoeol(window_it->second.get_window_reference());
        }
    }
    wclrtobot(window_it->second.get_window_reference());
    box(window_it->second.get_window_reference(), 0, 0);
    wrefresh(window_it->second.get_window_reference());
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_messagewindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();

    if (select_task_mode == true) {
        std::size_t width = window_it->second.get_screen_coordinates_pixel().width_pix;
        std::string str = message_text;
        if (str.size() > (width - 4)) {
            str = str.substr(0, width - 4) + "... ";
        }
        wattron(window_it->second.get_window_reference(), COLOR_PAIR(message_text_color));
        mvwprintw(window_it->second.get_window_reference(), 1, 1, str.c_str());
        wclrtoeol(window_it->second.get_window_reference());
        wattroff(window_it->second.get_window_reference(), COLOR_PAIR(message_text_color));
        box(window_it->second.get_window_reference(), 0, 0);
        wrefresh(window_it->second.get_window_reference());
    }

    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_diagnosticwindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    Color color;

    if (show_task_diagnostic_mode == false) {
        mvwprintw(window_it->second.get_window_reference(), 1, 1, "System Diagnostics:");
        wclrtoeol(window_it->second.get_window_reference());
        std::string dashed(window_it->second.get_screen_coordinates_pixel().width_pix - 2, '-');
        mvwprintw(window_it->second.get_window_reference(), 2, 1, dashed.c_str());
        std::string system_diagnostic_topic = "/srv_system_diagnostics";
        ros::ServiceClient system_diag_client =
            nodeHandle->serviceClient<eros::srv_get_diagnostics>(system_diagnostic_topic);
        eros::srv_get_diagnostics srv;
        srv.request.MinLevel = 0;
        srv.request.DiagnosticType = 0;
        if (system_diag_client.call(srv)) {
            for (std::size_t i = 0; i < srv.response.diag_list.size(); ++i) {
                Diagnostic::DiagnosticDefinition diag = convert(srv.response.diag_list.at(i));
                std::string str = "  " + Diagnostic::DiagnosticTypeString(diag.type);
                if (diag.message == Diagnostic::Message::NODATA) {
                    color = Color::NO_COLOR;
                }
                else {
                    switch (diag.level) {
                        case Level::Type::DEBUG: color = Color::NO_COLOR; break;
                        case Level::Type::INFO: color = Color::GREEN_COLOR; break;
                        case Level::Type::NOTICE: color = Color::GREEN_COLOR; break;
                        case Level::Type::WARN:
                            color = Color::YELLOW_COLOR;
                            str += ": " + diag.description;
                            break;
                        case Level::Type::ERROR:
                            color = Color::RED_COLOR;
                            str += ": " + diag.description;
                            break;
                        case Level::Type::FATAL:
                            color = Color::RED_COLOR;
                            str += ": " + diag.description;
                            break;
                        default: color = Color::RED_COLOR; break;
                    }
                }

                str += "  ";
                if (str.size() >
                    (std::size_t)(window_it->second.get_screen_coordinates_pixel().width_pix - 4)) {
                    str = str.substr(0,
                                     (std::size_t)(window_it->second.get_screen_coordinates_pixel()
                                                       .width_pix -
                                                   4)) +
                          "...";
                }
                wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
                mvwprintw(window_it->second.get_window_reference(), i + 3, 1, str.c_str());
                wclrtoeol(window_it->second.get_window_reference());
                wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
            }
        }
        else {
            color = Color::YELLOW_COLOR;
            wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
            mvwprintw(window_it->second.get_window_reference(),
                      3,
                      1,
                      "!!! No System Diagostics Available !!!");
            wclrtoeol(window_it->second.get_window_reference());
            wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        }
    }
    else {  // if (show_task_diagnostic_mode == true) {
        mvwprintw(window_it->second.get_window_reference(), 1, 1, "Node Diagnostics:");
        wclrtoeol(window_it->second.get_window_reference());
        std::string dashed(window_it->second.get_screen_coordinates_pixel().width_pix - 2, '-');
        mvwprintw(window_it->second.get_window_reference(), 2, 1, dashed.c_str());
        for (std::size_t i = 0; i < task_diagnostics_to_show.size(); ++i) {
            std::string str =
                "  " + Diagnostic::DiagnosticTypeString(task_diagnostics_to_show.at(i).type);
            switch (task_diagnostics_to_show.at(i).level) {
                case Level::Type::DEBUG: color = Color::NO_COLOR; break;
                case Level::Type::INFO: color = Color::GREEN_COLOR; break;
                case Level::Type::NOTICE: color = Color::GREEN_COLOR; break;
                case Level::Type::WARN:
                    color = Color::YELLOW_COLOR;
                    str += ": " + task_diagnostics_to_show.at(i).description;
                    break;
                case Level::Type::ERROR:
                    color = Color::RED_COLOR;
                    str += ": " + task_diagnostics_to_show.at(i).description;
                    break;
                case Level::Type::FATAL:
                    color = Color::RED_COLOR;
                    str += ": " + task_diagnostics_to_show.at(i).description;
                    break;
                default: color = Color::RED_COLOR; break;
            }
            str += "  ";
            if (str.size() >
                (std::size_t)(window_it->second.get_screen_coordinates_pixel().width_pix - 4)) {
                str = str.substr(
                          0,
                          (std::size_t)(window_it->second.get_screen_coordinates_pixel().width_pix -
                                        4)) +
                      "...";
            }
            wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
            mvwprintw(window_it->second.get_window_reference(), i + 3, 1, str.c_str());
            wclrtoeol(window_it->second.get_window_reference());
            wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        }
    }
    wclrtobot(window_it->second.get_window_reference());
    box(window_it->second.get_window_reference(), 0, 0);
    wrefresh(window_it->second.get_window_reference());
    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_devicewindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();
    std::map<std::string, Device>::iterator device_it;
    std::map<uint16_t, std::string>::iterator device_id_it;
    const uint16_t DEVICESTART_COORD_Y = 1;
    const uint16_t DEVICESTART_COORD_X = 1;
    uint16_t devices_shown = 0;
    uint16_t index = 0;
    uint16_t start_device_index = 0;
    for (uint16_t i = start_device_index; i < (uint16_t)device_list.size(); ++i) {
        device_id_it = device_name_list.find(i);
        if (device_id_it == device_name_list.end()) {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::DATA_STORAGE,
                Level::Type::ERROR,
                Diagnostic::Message::DIAGNOSTIC_FAILED,
                "Device List does not contain ID: " + std::to_string(i));
            return diag;
        }
        std::string key = device_id_it->second;
        device_it = device_list.find(key);
        if (device_it == device_list.end()) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Device List does not contain ID: " + key);
            return diag;
        }

        Color color = Color::UNKNOWN;
        switch (device_it->second.state) {
            case Node::State::UNKNOWN: color = Color::RED_COLOR; break;
            case Node::State::START: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZING: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZED: color = Color::YELLOW_COLOR; break;
            case Node::State::NODATA: color = Color::RED_COLOR; break;
            case Node::State::RUNNING: color = Color::BLUE_COLOR; break;
            case Node::State::PAUSED: color = Color::GREEN_COLOR; break;
            case Node::State::RESET: color = Color::YELLOW_COLOR; break;
            case Node::State::FINISHED: color = Color::YELLOW_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }
        std::string str = get_device_info(device_it->second, false);
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
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_taskwindow(
    std::map<std::string, WindowManager>::iterator window_it) {
    Diagnostic::DiagnosticDefinition diag = diagnostic_helper.get_root_diagnostic();

    if (task_list.size() == 0) {
        diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::SOFTWARE,
                                                   Level::Type::NOTICE,
                                                   Diagnostic::Message::NOERROR,
                                                   "No Tasks Defined Yet.");
    }
    select_task_mode = true;  //
    if ((select_task_mode == true) and (selected_task_index == -1)) {
        selected_task_index = 0;
    }
    int key_pressed = wgetch(window_it->second.get_window_reference());
    if ((key_pressed == KEY_q) || (key_pressed == KEY_Q)) {
        kill_me = true;
    }
    else if ((key_pressed == KEY_c) || (key_pressed == KEY_C)) {
        set_message_text("Clearing All Snapshots...", Level::Type::WARN);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::CLEAR_SNAPSHOTS;
        command_pub.publish(command);
    }
    else if ((key_pressed == KEY_s) || (key_pressed == KEY_S)) {
        set_message_text("Requesting System Snapshot...", Level::Type::INFO);
        eros::command command;
        command.stamp = ros::Time::now();
        command.Command = (uint16_t)Command::Type::GENERATE_SNAPSHOT;
        command.Option1 = (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER;
        command_pub.publish(command);
    }
    else if (key_pressed == KEY_UP) {
        if (select_task_mode == true) {
            if (start_node_index > 0) {
                --start_node_index;
            }
            if (selected_task_index > 0) {
                --selected_task_index;
            }
            else {
                selected_task_index = 0;
            }
            if (selected_task_index == 0) {
                start_node_index = 0;
            }
        }
    }
    else if (key_pressed == KEY_DOWN) {
        if (select_task_mode == true) {
            if (selected_task_index < (((int16_t)task_list.size() - 1))) {
                ++selected_task_index;
            }
            else {
                selected_task_index = 0;
                start_node_index = 0;
            }
            if (selected_task_index >= task_list_max_rows) {
                start_node_index++;
            }
        }
    }
    else if ((key_pressed == KEY_f) || (key_pressed == KEY_F)) {
        if (select_task_mode == true) {
            std::map<uint16_t, std::string>::iterator task_name_lookup =
                task_name_list.find(selected_task_index);
            if (task_name_lookup == task_name_list.end()) {
                std::string str = "Unable to lookup Task: " + std::to_string(selected_task_index);
                logger->log_error(str);
                set_message_text(str, Color::RED_COLOR);
            }
            else {
                std::map<std::string, Task>::iterator task_info_it =
                    task_list.find(task_name_lookup->second);
                if (task_info_it == task_list.end()) {
                    std::string str = "Unable to lookup Task: " + task_name_lookup->second;
                    set_message_text(str, Color::RED_COLOR);
                    logger->log_error(str);
                }
                else {
                    if (task_info_it->second.type == SystemMonitorProcess::TaskType::EROS) {
                        std::string firmware_topic =
                            task_info_it->second.node_name + "/srv_firmware";
                        if (nodeHandle == nullptr) {
                            logger->log_error("Node Handle has no memory!");
                        }
                        ros::ServiceClient client =
                            nodeHandle->serviceClient<eros::srv_firmware>(firmware_topic);
                        eros::srv_firmware srv;
                        if (client.call(srv)) {
                            set_message_text(
                                "Firmware: Node: " + srv.response.NodeName +
                                    " Version: " + std::to_string(srv.response.MajorRelease) + "." +
                                    std::to_string(srv.response.MinorRelease) + "." +
                                    std::to_string(srv.response.BuildNumber) +
                                    " Desc: " + srv.response.Description,
                                Color::NO_COLOR);
                        }
                        else {
                            std::string str = "Node: " + task_info_it->second.node_name +
                                              " Firmware Change Failed!";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                        }
                    }
                    else {
                        std::string str =
                            "Node: " + task_info_it->second.node_name + " is not an EROS Node.";
                        set_message_text(str, Color::YELLOW_COLOR);
                        logger->log_warn(str);
                    }
                }
            }
        }
    }
    else if ((key_pressed == KEY_l) || (key_pressed == KEY_L)) {
        change_log_level_mode = true;
        change_nodestate_mode = false;
    }
    else if ((key_pressed == KEY_n) || (key_pressed == KEY_N)) {
        change_log_level_mode = false;
        change_nodestate_mode = true;
    }
    else if ((key_pressed == KEY_d) || (key_pressed == KEY_D)) {
        if (select_task_mode == true) {
            show_task_diagnostic_mode = !show_task_diagnostic_mode;
            std::map<uint16_t, std::string>::iterator task_name_lookup =
                task_name_list.find(selected_task_index);
            if (task_name_lookup == task_name_list.end()) {
                std::string str = "Unable to lookup Task: " + std::to_string(selected_task_index);
                logger->log_error(str);
                set_message_text(str, Color::RED_COLOR);
                task_diagnostics_to_show.clear();
            }
            else {
                std::map<std::string, Task>::iterator task_info_it =
                    task_list.find(task_name_lookup->second);
                if (task_info_it == task_list.end()) {
                    std::string str = "Unable to lookup Task: " + task_name_lookup->second;
                    set_message_text(str, Color::RED_COLOR);
                    logger->log_error(str);
                    task_diagnostics_to_show.clear();
                }
                else {
                    if (task_info_it->second.type == SystemMonitorProcess::TaskType::EROS) {
                        std::string diagnostics_topic =
                            task_info_it->second.node_name + "/srv_diagnostics";
                        if (nodeHandle == nullptr) {
                            logger->log_error("Node Handle has no memory!");
                        }
                        ros::ServiceClient client =
                            nodeHandle->serviceClient<eros::srv_get_diagnostics>(diagnostics_topic);
                        eros::srv_get_diagnostics srv;
                        srv.request.MinLevel = 0;
                        srv.request.DiagnosticType = 0;
                        if (client.call(srv)) {
                            std::vector<Diagnostic::DiagnosticDefinition> diag_list;
                            for (std::size_t i = 0; i < srv.response.diag_list.size(); ++i) {
                                diag_list.push_back(convert(srv.response.diag_list.at(i)));
                            }
                            task_diagnostics_to_show = diag_list;
                            set_message_text("Diagnostics for Node: " +
                                                 task_info_it->second.node_name + " Received.",
                                             Color::NO_COLOR);
                        }
                        else {
                            std::string str = "Node: " + task_info_it->second.node_name +
                                              " Diagnostics Retreival Failed!";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                            task_diagnostics_to_show.clear();
                        }
                    }
                    else {
                        std::string str =
                            "Node: " + task_info_it->second.node_name + " is not an EROS Node.";
                        set_message_text(str, Color::YELLOW_COLOR);
                        logger->log_warn(str);
                        task_diagnostics_to_show.clear();
                    }
                }
            }
        }
    }
    else if ((key_pressed == KEY_1) || (key_pressed == KEY_2) || (key_pressed == KEY_3) ||
             (key_pressed == KEY_4) || (key_pressed == KEY_5) || (key_pressed == KEY_6) ||
             (key_pressed == KEY_7) || (key_pressed == KEY_8) || (key_pressed == 9)) {
        std::map<uint16_t, std::string>::iterator task_name_lookup =
            task_name_list.find(selected_task_index);
        if (task_name_lookup == task_name_list.end()) {
            std::string str = "Unable to lookup Task: " + std::to_string(selected_task_index);
            logger->log_error(str);
            set_message_text(str, Color::RED_COLOR);
        }
        else {
            std::map<std::string, Task>::iterator task_info_it =
                task_list.find(task_name_lookup->second);
            if (task_info_it == task_list.end()) {
                std::string str = "Unable to lookup Task: " + task_name_lookup->second;
                set_message_text(str, Color::RED_COLOR);
                logger->log_error(str);
            }
            else {
                if (task_info_it->second.type == SystemMonitorProcess::TaskType::EROS) {
                    if (change_log_level_mode == true) {
                        std::string logger_level_topic =
                            task_info_it->second.node_name + "/srv_loggerlevel";
                        if (nodeHandle == nullptr) {
                            logger->log_error("Node Handle has no memory!");
                        }
                        ros::ServiceClient client =
                            nodeHandle->serviceClient<eros::srv_logger_level>(logger_level_topic);
                        eros::srv_logger_level srv;
                        uint16_t verbosity_value = key_pressed - (KEY_1) + 1;
                        std::string verbosity_level =
                            Level::LevelString((Level::Type)verbosity_value);
                        if (verbosity_level == "UNKNOWN") {
                            std::string str = "Requested Log Level Not Supported.";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                            change_log_level_mode = false;
                        }
                        srv.request.LoggerLevel = verbosity_level;
                        if (client.call(srv)) {
                            std::string str = "Node: " + task_info_it->second.node_name + " " +
                                              srv.response.Response;
                            set_message_text(str, Color::NO_COLOR);
                            change_log_level_mode = false;
                        }
                        else {
                            std::string str = "Node: " + task_info_it->second.node_name +
                                              " Logger Level Change Failed!";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                            change_log_level_mode = false;
                        }
                    }
                    else if (change_nodestate_mode == true) {
                        std::string nodestate_topic =
                            task_info_it->second.node_name + "/srv_nodestate_change";
                        if (nodeHandle == nullptr) {
                            logger->log_error("Node Handle has no memory!");
                        }
                        ros::ServiceClient client =
                            nodeHandle->serviceClient<eros::srv_change_nodestate>(nodestate_topic);
                        eros::srv_change_nodestate srv;
                        uint16_t state_value = key_pressed - (KEY_1) + 1;
                        std::string req_state = Node::NodeStateString((Node::State)state_value);
                        if (req_state == "UNKNOWN") {
                            std::string str = "Requested Node State Not Supported.";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                            change_nodestate_mode = false;
                        }
                        srv.request.RequestedNodeState = req_state;
                        if (client.call(srv)) {
                            if (req_state == srv.response.NodeState) {
                                std::string str = "Node: " + task_info_it->second.node_name +
                                                  " New Node State: " + srv.response.NodeState;
                                set_message_text(str, Color::NO_COLOR);
                            }
                            else {
                                std::string str = "Node: " + task_info_it->second.node_name +
                                                  " Requested State: " + req_state + " Rejected! ";
                                set_message_text(str, Color::YELLOW_COLOR);
                            }
                            change_nodestate_mode = false;
                        }
                        else {
                            std::string str = "Node: " + task_info_it->second.node_name +
                                              " Node State Change Request Failed! ";
                            set_message_text(str, Color::YELLOW_COLOR);
                            logger->log_warn(str);
                            logger->log_warn(str);
                            change_nodestate_mode = false;
                        }
                    }
                }
                else {
                    std::string str =
                        "Node: " + task_info_it->second.node_name + " is not an EROS Node.";
                    set_message_text(str, Color::YELLOW_COLOR);
                    logger->log_warn(str);
                }
            }
        }
    }

    std::map<std::string, Task>::iterator task_it;  // = task_list.begin();
    std::map<uint16_t, std::string>::iterator task_id_it;
    const uint16_t TASKSTART_COORD_Y = 1;
    const uint16_t TASKSTART_COORD_X = 1;
    uint16_t tasks_shown = 0;
    uint16_t index = 0;
    for (uint16_t i = start_node_index; i < (uint16_t)task_list.size(); ++i) {
        if (tasks_shown >= task_list_max_rows) {
            break;
        }
        task_id_it = task_name_list.find(i);
        if (task_id_it == task_name_list.end()) {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::DATA_STORAGE,
                Level::Type::ERROR,
                Diagnostic::Message::DIAGNOSTIC_FAILED,
                "Task List does not contain ID: " + std::to_string(i));
            return diag;
        }
        std::string key = task_id_it->second;
        task_it = task_list.find(key);
        if (task_it == task_list.end()) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::DATA_STORAGE,
                                                       Level::Type::ERROR,
                                                       Diagnostic::Message::DIAGNOSTIC_FAILED,
                                                       "Task List does not contain ID: " + key);
            return diag;
        }
        Color color = Color::UNKNOWN;
        switch (task_it->second.state) {
            case Node::State::UNKNOWN: color = Color::RED_COLOR; break;
            case Node::State::START: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZING: color = Color::YELLOW_COLOR; break;
            case Node::State::INITIALIZED: color = Color::YELLOW_COLOR; break;
            case Node::State::NODATA: color = Color::RED_COLOR; break;
            case Node::State::RUNNING: color = Color::BLUE_COLOR; break;
            case Node::State::PAUSED: color = Color::GREEN_COLOR; break;
            case Node::State::RESET: color = Color::YELLOW_COLOR; break;
            case Node::State::FINISHED: color = Color::YELLOW_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }

        wattron(window_it->second.get_window_reference(), COLOR_PAIR(color));
        std::string str = get_task_info(task_it->second, (i == selected_task_index));
        mvwprintw(window_it->second.get_window_reference(),
                  TASKSTART_COORD_Y + 2 + (int)index,
                  TASKSTART_COORD_X + 1,
                  str.c_str());
        wclrtoeol(window_it->second.get_window_reference());
        wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        tasks_shown++;
        index++;
    }
    box(window_it->second.get_window_reference(), 0, 0);
    wrefresh(window_it->second.get_window_reference());
    return diag;
}
std::string SystemMonitorProcess::get_task_info(Task task, bool selected) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = task_window_fields.find(TaskFieldColumn::MARKER)->second.width;
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
        width = task_window_fields.find(TaskFieldColumn::ID)->second.width;
        std::string tempstr = std::to_string(task.id);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::HOSTNAME)->second.width;
        std::string tempstr = task.host_device;
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
        width = task_window_fields.find(TaskFieldColumn::NODENAME)->second.width;
        std::string tempstr = task.node_name;
        std::size_t found_hostname = task.node_name.find(task.host_device);
        if (found_hostname != std::string::npos) {
            tempstr.replace(found_hostname, task.host_device.length(), "");
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
        width = task_window_fields.find(TaskFieldColumn::STATUS)->second.width;
        std::string tempstr = Node::NodeStateString(task.state);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::RESTARTS)->second.width;
        std::string tempstr = std::to_string(task.restart_count);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::PID)->second.width;
        std::string tempstr = std::to_string(task.pid);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::CPU)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", task.cpu_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::RAM)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", task.mem_used_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::RX)->second.width;
        std::string max_number_str(width - 4, '9');
        double max_num = std::atof(max_number_str.c_str()) + 0.99;
        if (task.last_heartbeat_delta > max_num) {
            task.last_heartbeat_delta = max_num;
        }
        char tempstr[2 * width];
        sprintf(tempstr, "%2.2f", task.last_heartbeat_delta);
        std::string tempstr_str = std::string(tempstr);
        std::size_t spaces = width - tempstr_str.size();
        if (spaces > 0) {
            tempstr_str += std::string(spaces, ' ');
        }
        str += tempstr_str;
    }

    return str;
}
std::string SystemMonitorProcess::get_device_info(Device device, bool selected) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = device_window_fields.find(DeviceFieldColumn::MARKER)->second.width;
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
        width = device_window_fields.find(DeviceFieldColumn::ID)->second.width;
        std::string tempstr = std::to_string(device.id);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::NAME)->second.width;
        std::string tempstr = device.name;
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
        width = device_window_fields.find(DeviceFieldColumn::CPU)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.cpu_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }

    {
        width = device_window_fields.find(DeviceFieldColumn::RAM)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.ram_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::DISK)->second.width;
        char c_tempstr[8];
        sprintf(c_tempstr, "%3.2f", device.disk_av_perc);
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::LOADFACTOR)->second.width;
        char c_tempstr[128];
        sprintf(c_tempstr,
                "[%3.2f,%3.2f,%3.2f]",
                device.load_factor.at(0),
                device.load_factor.at(1),
                device.load_factor.at(2));
        std::string tempstr = std::string(c_tempstr);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = device_window_fields.find(DeviceFieldColumn::RX)->second.width;
        std::string max_number_str(width - 4, '9');
        double max_num = std::atof(max_number_str.c_str()) + 0.99;
        if (device.last_heartbeat_delta > max_num) {
            device.last_heartbeat_delta = max_num;
        }
        char tempstr[2 * width];
        sprintf(tempstr, "%2.2f", device.last_heartbeat_delta);
        std::string tempstr_str = std::string(tempstr);
        std::size_t spaces = width - tempstr_str.size();
        if (spaces > 0) {
            tempstr_str += std::string(spaces, ' ');
        }
        str += tempstr_str;
    }

    return str;
}
