#include "SystemMonitorProcess.h"
WINDOW *create_newwin(int height, int width, int starty, int startx) {
    WINDOW *local_win;

    local_win = newwin(height, width, starty, startx);
    box(local_win, 0, 0); /* 0, 0 gives default characters
                           * for the vertical and horizontal
                           * lines			*/
    wrefresh(local_win);  /* Show that box 		*/

    return local_win;
}
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

    std::map<std::string, Task>::iterator task_it = task_list.begin();
    while (task_it != task_list.end()) {
        task_it->second.last_heartbeat_delta += t_dt;
        if (task_it->second.last_heartbeat_delta >= COMMTIMEOUT_THRESHOLD) {
            task_it->second.state = Node::State::NODATA;
        }

        ++task_it;
    }

    std::map<std::string, WindowManager>::iterator win_it = windows.begin();
    while (win_it != windows.end()) {
        if (win_it->first == "header") {}
        else if (win_it->first == "task_window") {
            diag = update_taskwindow(win_it);
            if (diag.level > Level::Type::WARN) {
                return diag;
            }
        }
        else {
            logger->log_warn("Window: " + win_it->first + " Not Supported.");
        }
        wrefresh(win_it->second.get_window_reference());
        ++win_it;
    }

    return diag;
}
Diagnostic::DiagnosticDefinition SystemMonitorProcess::update_nodelist(
    std::vector<std::string> node_list,
    std::vector<std::string> heartbeat_list,
    std::vector<std::string> &new_heartbeat_topics_to_subscribe) {
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
            }
            else if (task_it->second.type == TaskType::NON_EROS) {
                task_it->second.type = TaskType::EROS;
                new_heartbeat_topics_to_subscribe.push_back(*it);
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
std::vector<Diagnostic::DiagnosticDefinition> SystemMonitorProcess::new_commandmsg(
    const eros::command::ConstPtr &t_msg) {
    (void)t_msg;  // Not currently used.
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
        WindowManager window("status_window", 0, 72.0, 30.0, 25.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }
    {
        WindowManager window("instruction_window", 30, 72.0, 40.0, 25.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }
    {
        WindowManager window("device_window", 70, 72.0, 30.0, 25.0);
        std::pair<std::string, WindowManager> newwin = std::make_pair(window.get_name(), window);
        windows.insert(newwin);
    }

    // for (auto &window : windows) {
    std::map<std::string, WindowManager>::iterator it = windows.begin();
    while (it != windows.end()) {
        WindowManager::ScreenCoordinatePixel coord_pix = convertCoordinate(
            it->second.get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);

        WINDOW *win = create_newwin(coord_pix.height_pix,
                                    coord_pix.width_pix,
                                    coord_pix.start_y_pix,
                                    coord_pix.start_x_pix);

        it->second.set_window_reference(win);

        if (it->first == "header") {
            wbkgd(it->second.get_window_reference(), COLOR_PAIR(Color::NO_COLOR));
            keypad(it->second.get_window_reference(), TRUE);
        }
        else if (it->first == "task_window") {
            keypad(it->second.get_window_reference(), TRUE);
            std::string header = get_taskheader();
            mvwprintw(it->second.get_window_reference(), 1, 1, header.c_str());
            std::string dashed(header.size(), '-');
            mvwprintw(it->second.get_window_reference(), 2, 1, dashed.c_str());
        }
        else {
            logger->log_warn("Window: " + it->first + " Not Supported.");
        }
        wrefresh(it->second.get_window_reference());
        ++it;
    }
    return true;
}
std::string SystemMonitorProcess::get_taskheader() {
    std::string str = "";
    std::map<TaskFieldColumn, TaskField>::iterator it = task_window_fields.begin();
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
    return str.c_str();
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
    std::map<std::string, Task>::iterator task_it;  // = task_list.begin();
    std::map<uint16_t, std::string>::iterator task_id_it = task_name_list.begin();
    int index = 0;
    const uint16_t TASKSTART_COORD_Y = 1;
    const uint16_t TASKSTART_COORD_X = 1;
    while (task_id_it != task_name_list.end()) {
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
        std::string str = get_task_info(task_it->second);
        mvwprintw(window_it->second.get_window_reference(),
                  TASKSTART_COORD_Y + 3 + (int)index,
                  TASKSTART_COORD_X + 1,
                  str.c_str());
        wclrtoeol(window_it->second.get_window_reference());
        wattroff(window_it->second.get_window_reference(), COLOR_PAIR(color));
        index++;
        ++task_id_it;
    }
    box(window_it->second.get_window_reference(), 0, 0);
    wrefresh(window_it->second.get_window_reference());
    return diag;
}
std::string SystemMonitorProcess::get_task_info(Task task) {
    std::string str = "";
    std::size_t width = 0;
    {
        width = task_window_fields.find(TaskFieldColumn::MARKER)->second.width;
        for (std::size_t i = 0; i < width; ++i) { str += " "; }
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
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::NODENAME)->second.width;
        std::string tempstr = task.node_name;
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
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
        std::string tempstr = std::to_string(task.cpu_used_perc);
        std::size_t spaces = width - tempstr.size();
        if (spaces > 0) {
            tempstr += std::string(spaces, ' ');
        }
        str += tempstr;
    }
    {
        width = task_window_fields.find(TaskFieldColumn::RAM)->second.width;
        std::string tempstr = std::to_string(task.mem_used_perc);
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
