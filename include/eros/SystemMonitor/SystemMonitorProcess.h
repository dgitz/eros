/*! \file SystemMonitorProcess.h
 */
#ifndef SYSTEMMONITORPROCESS_h
#define SYSTEMMONITORPROCESS_h
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <curses.h>
#include <eros/BaseNodeProcess.h>
#include <eros/SnapshotNode/SnapshotProcess.h>
#include <eros/heartbeat.h>
#include <ros/ros.h>
WINDOW* create_newwin(int height, int width, int starty, int startx);
/*! \class WindowManager SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief WindowManager handles the coordinates and reference to the WINDOW object. */
class WindowManager
{
   public:
    struct ScreenCoordinatePerc {
        ScreenCoordinatePerc(double start_x, double start_y, double width, double height)
            : start_x_perc(start_x), start_y_perc(start_y), width_perc(width), height_perc(height) {
        }
        double start_x_perc;
        double start_y_perc;
        double width_perc;
        double height_perc;
    };
    struct ScreenCoordinatePixel {
        ScreenCoordinatePixel(double start_x, double start_y, double width, double height)
            : start_x_pix(start_x), start_y_pix(start_y), width_pix(width), height_pix(height) {
        }
        uint16_t start_x_pix;
        uint16_t start_y_pix;
        uint16_t width_pix;
        uint16_t height_pix;
    };
    WindowManager(const std::string _name,
                  double start_x_perc,
                  double start_y_perc,
                  double width_perc,
                  double height_perc)
        : name(_name),
          screen_coord_perc(start_x_perc, start_y_perc, width_perc, height_perc),
          screen_coord_pixel(0, 0, 0, 0),
          window_reference(nullptr) {
    }
    ~WindowManager() {
    }
    std::string get_name() {
        return name;
    }
    void set_screen_coordinates_pix(ScreenCoordinatePixel coord) {
        screen_coord_pixel = coord;
    }
    ScreenCoordinatePerc get_screen_coordinates_perc() {
        return screen_coord_perc;
    }
    ScreenCoordinatePixel get_screen_coordinates_pixel() {
        return screen_coord_pixel;
    }
    std::string pretty() {
        if (window_reference == nullptr) {
            return name + " Is Uninitialized.";
        }
        else {
            char tempstr[128];
            sprintf(tempstr,
                    "%s (X:%d%%Y:%d%%W:%d%%H:%d%%)",
                    name.c_str(),
                    (uint16_t)screen_coord_perc.start_x_perc,
                    (uint16_t)screen_coord_perc.start_y_perc,
                    (uint16_t)screen_coord_perc.width_perc,
                    (uint16_t)screen_coord_perc.height_perc);
            return std::string(tempstr);
        }
    }
    void set_window_reference(WINDOW* win) {
        window_reference = win;
    }
    WINDOW* get_window_reference() {
        return window_reference;
    }

   private:
    std::string name;
    ScreenCoordinatePerc screen_coord_perc;
    ScreenCoordinatePixel screen_coord_pixel;
    WINDOW* window_reference;
};
/*! \class SystemMonitorProcess SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief  */
class SystemMonitorProcess : public BaseNodeProcess
{
   public:
    const bool DEBUG_MODE = false;
    /*! \brief How long in seconds before marking a Node as Timed Out.*/
    const double COMMTIMEOUT_THRESHOLD = 5.0f;
    /*! \brief The minimum width in pixels of the Main Window.*/
    const uint16_t MINWINDOW_WIDTH = 140;
    /*! \brief The minimum height in pixels of the Main Window.*/
    const uint16_t MINWINDOW_HEIGHT = 240;

    /*! \brief The amount of time to show text in the message window.*/
    const double TIME_TO_SHOW_MESSAGES = 10.0f;  // Seconds

    // Keys
    static constexpr int KEY_q = 113;
    static constexpr int KEY_Q = 81;
    static constexpr int KEY_s = 83;
    static constexpr int KEY_S = 115;
    static constexpr int KEY_c = 99;
    static constexpr int KEY_C = 67;
    static constexpr int KEY_f = 102;
    static constexpr int KEY_F = 70;
    static constexpr int KEY_g = 103;
    static constexpr int KEY_G = 71;
    static constexpr int KEY_l = 108;
    static constexpr int KEY_L = 76;
    static constexpr int KEY_d = 100;
    static constexpr int KEY_D = 68;
    static constexpr int KEY_r = 114;
    static constexpr int KEY_R = 82;
    static constexpr int KEY_p = 112;
    static constexpr int KEY_P = 80;
    static constexpr int KEY_m = 109;
    static constexpr int KEY_M = 77;
    static constexpr int KEY_n = 110;
    static constexpr int KEY_N = 78;

    static constexpr int KEY_1 = 49;
    static constexpr int KEY_2 = 50;
    static constexpr int KEY_3 = 51;
    static constexpr int KEY_4 = 52;
    static constexpr int KEY_5 = 53;
    static constexpr int KEY_6 = 54;
    static constexpr int KEY_7 = 55;
    static constexpr int KEY_8 = 56;
    static constexpr int KEY_9 = 57;

    enum class Color {
        UNKNOWN = 0,
        NO_COLOR = 1,
        RED_COLOR = 2,
        YELLOW_COLOR = 3,
        GREEN_COLOR = 4,
        BLUE_COLOR = 5,
        GRAY_COLOR = 6,
        PURPLE_COLOR = 7,
        END_OF_LIST = 8
    };
    enum class TaskType { UNKNOWN = 0, EROS = 1, NON_EROS = 2 };
    enum class TaskFieldColumn {
        MARKER = 0,
        ID = 1,
        HOSTNAME = 2,
        NODENAME = 3,
        STATUS = 4,
        RESTARTS = 5,
        PID = 6,
        CPU = 7,
        RAM = 8,
        RX = 9
    };
    enum class DeviceFieldColumn {
        MARKER = 0,
        ID = 1,
        NAME = 2,
        CPU = 3,
        RAM = 4,
        DISK = 5,
        LOADFACTOR = 6,
        RX = 7
    };
    struct Task {
        Task(uint16_t _id,
             TaskType _type,
             std::string _host_device,
             std::string _base_node_name,
             std::string _node_name)
            : id(_id),
              state(Node::State::START),
              type(_type),
              pid(0),
              host_device(_host_device),
              base_node_name(_base_node_name),
              node_name(_node_name),
              cpu_used_perc(0.0),
              mem_used_perc(0.0),
              last_heartbeat(0.0),
              last_heartbeat_delta(0.0),
              restart_count(0) {
        }
        bool initialized;
        uint16_t id;
        Node::State state;
        TaskType type;
        uint16_t pid;
        std::string host_device;
        std::string base_node_name;
        std::string node_name;
        double cpu_used_perc;
        double mem_used_perc;
        double last_heartbeat;
        double last_heartbeat_delta;
        uint64_t restart_count;
    };
    struct Device {
        Device(uint16_t _id, std::string _name)
            : id(_id),
              name(_name),
              state(Node::State::INITIALIZING),
              last_heartbeat_delta(0.0),
              cpu_av_perc(0.0),
              ram_av_perc(0.0),
              disk_av_perc(0.0),
              load_factor({0.0, 0.0, 0.0}) {
        }
        uint16_t id;
        bool initialized;
        std::string name;
        Node::State state;
        double last_heartbeat_delta;
        double cpu_av_perc;
        double ram_av_perc;
        double disk_av_perc;
        std::vector<double> load_factor;
    };

    struct Field {
        Field(std::string _text, uint16_t _width) : text(_text), width(_width) {
        }
        std::string text;
        std::size_t width;
    };
    SystemMonitorProcess()
        : kill_me(false),
          nodeHandle(nullptr),
          mainwindow_width(0),
          mainwindow_height(0),
          select_task_mode(false),
          change_log_level_mode(false),
          show_task_diagnostic_mode(false),
          change_nodestate_mode(false),
          select_device_mode(false),
          selected_task_index(-1),
          start_node_index(0),
          task_list_max_rows(5),
          timer_showing_message_in_window(0.0),
          message_text(""),
          message_text_color(Color::NO_COLOR) {
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::MARKER, Field("", 3)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::ID, Field("ID", 4)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::HOSTNAME, Field(" Host ", 20)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::NODENAME, Field(" NodeName ", 30)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::STATUS, Field(" Status ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::RESTARTS, Field(" Restarts ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::PID, Field(" PID ", 8)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::CPU, Field(" CPU(%) ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::RAM, Field(" RAM(%) ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, Field>(TaskFieldColumn::RX, Field(" Rx ", 6)));

        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::MARKER, Field("", 3)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::ID, Field("ID", 3)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::NAME, Field(" Device ", 25)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::CPU, Field(" CPU Av ", 8)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::RAM, Field(" RAM Av ", 8)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::DISK, Field(" DISK Av ", 8)));
        device_window_fields.insert(std::pair<DeviceFieldColumn, Field>(
            DeviceFieldColumn::LOADFACTOR, Field(" LOAD FACTOR ", 18)));
        device_window_fields.insert(
            std::pair<DeviceFieldColumn, Field>(DeviceFieldColumn::RX, Field(" Rx ", 6)));
    }
    ~SystemMonitorProcess();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    bool initialize_windows();
    bool set_nodeHandle(ros::NodeHandle* nh) {
        nodeHandle = nh;
        std::string systemcommand_topic = "/SystemCommand";
        command_pub = nodeHandle->advertise<eros::command>(systemcommand_topic, 1);
        return true;
    }

    // Update Functions
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    Diagnostic::DiagnosticDefinition update_headerwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_taskwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_instructionwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_messagewindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_diagnosticwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_devicewindow(
        std::map<std::string, WindowManager>::iterator it);

    Diagnostic::DiagnosticDefinition update_nodelist(
        std::vector<std::string> node_list,
        std::vector<std::string> heartbeat_list,
        std::vector<std::string>& new_heartbeat_topics_to_subscribe,
        std::vector<std::string>& new_resourceused_topics_to_subscribe);
    std::map<std::string, Task> get_task_list() {
        return task_list;
    }

    Diagnostic::DiagnosticDefinition update_devicelist(
        std::vector<std::string> loadfactor_list,
        std::vector<std::string>& new_resourceavailable_topics_to_subscribe,
        std::vector<std::string>& new_loadfactor_topics_to_subscribe);
    std::map<std::string, Device> get_device_list() {
        return device_list;
    }

    // Message Functions
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    Diagnostic::DiagnosticDefinition new_commandstate(const eros::command_state::ConstPtr& t_msg) {
        eros::command_state state = convert_fromptr(t_msg);
        Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        if ((state.CurrentCommand.Command == (uint16_t)Command::Type::GENERATE_SNAPSHOT)) {
            if (state.CurrentCommand.Option1 ==
                (uint16_t)Command::GenerateSnapshot_Option1::RUN_MASTER) {
                if (state.State == (uint8_t)SnapshotProcess::SnapshotState::RUNNING) {
                    char tempstr[512];
                    sprintf(tempstr, "System Snap Progress: %4.2f %%", state.PercentComplete);
                    set_message_text(std::string(tempstr), Level::Type::NOTICE);
                }
                else if (state.State == (uint8_t)SnapshotProcess::SnapshotState::INCOMPLETE) {
                    set_message_text("System Snapshot Incomplete", Level::Type::WARN);
                }
                else if (state.State == (uint8_t)SnapshotProcess::SnapshotState::COMPLETE) {
                    set_message_text("System Snapshot Completed.", Level::Type::NOTICE);
                }
            }
        }
        else {
            set_message_text(state.diag.Description, (Level::Type)state.diag.Level);
        }

        return diag;
    }
    Diagnostic::DiagnosticDefinition new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_msg) {
        eros::heartbeat msg = convert_fromptr(t_msg);
        Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        if (update_task_list(msg) == true) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::INFO,
                                                       Diagnostic::Message::NOERROR,
                                                       "Updated Hearbeat.");
        }
        else {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::COMMUNICATIONS,
                Level::Type::WARN,
                Diagnostic::Message::DROPPING_PACKETS,
                "Unable to update Hearbeat: " + msg.HostName + " " + msg.NodeName);
            logger->log_diagnostic(diag);
        }
        return diag;
    }
    Diagnostic::DiagnosticDefinition new_resourceusedmessage(
        const eros::resource::ConstPtr& t_msg) {
        eros::resource msg = convert_fromptr(t_msg);
        Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        if (update_task_list(msg) == true) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::INFO,
                                                       Diagnostic::Message::NOERROR,
                                                       "Updated Resource Used.");
        }
        else {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::WARN,
                                                       Diagnostic::Message::DROPPING_PACKETS,
                                                       "Unable to update Resource: " + msg.Name);
            logger->log_diagnostic(diag);
        }
        return diag;
    }
    Diagnostic::DiagnosticDefinition new_resourceavailablemessage(
        const eros::resource::ConstPtr& t_msg) {
        eros::resource msg = convert_fromptr(t_msg);
        Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        if (update_device_list(msg) == true) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::INFO,
                                                       Diagnostic::Message::NOERROR,
                                                       "Updated Resource Available.");
        }
        else {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::WARN,
                                                       Diagnostic::Message::DROPPING_PACKETS,
                                                       "Unable to update Resource: " + msg.Name);
            logger->log_diagnostic(diag);
        }
        return diag;
    }
    Diagnostic::DiagnosticDefinition new_loadfactormessage(
        const eros::loadfactor::ConstPtr& t_msg) {
        eros::loadfactor msg = convert_fromptr(t_msg);
        Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        if (update_device_list(msg) == true) {
            diag = diagnostic_helper.update_diagnostic(Diagnostic::DiagnosticType::COMMUNICATIONS,
                                                       Level::Type::INFO,
                                                       Diagnostic::Message::NOERROR,
                                                       "Updated LoadFactor.");
        }
        else {
            diag = diagnostic_helper.update_diagnostic(
                Diagnostic::DiagnosticType::COMMUNICATIONS,
                Level::Type::WARN,
                Diagnostic::Message::DROPPING_PACKETS,
                "Unable to update LoadFactor: " + msg.DeviceName);
            logger->log_diagnostic(diag);
        }
        return diag;
    }
    void set_message_text(std::string text, Level::Type level) {
        if (text == "") {
            return;
        }
        Color color;
        switch (level) {
            case Level::Type::DEBUG: color = Color::NO_COLOR; break;
            case Level::Type::INFO: color = Color::NO_COLOR; break;
            case Level::Type::NOTICE: color = Color::GREEN_COLOR; break;
            case Level::Type::WARN: color = Color::YELLOW_COLOR; break;
            case Level::Type::ERROR: color = Color::RED_COLOR; break;
            case Level::Type::FATAL: color = Color::RED_COLOR; break;
            default: color = Color::RED_COLOR; break;
        }
        set_message_text(text, color);
    }
    void set_message_text(std::string text, Color color) {
        message_text = text;
        message_text_color = color;
        timer_showing_message_in_window = 0.0;
    }

    // Attribute Functions
    void update_armedstate(ArmDisarm::State v) {
        armed_state = v;
    }
    bool set_mainwindow(uint16_t t_mainwindow_width, uint16_t t_mainwindow_height) {
        mainwindow_width = t_mainwindow_width;
        mainwindow_height = t_mainwindow_height;
        if (mainwindow_width < MINWINDOW_WIDTH) {
            return false;
        }
        return true;
    }

    // Support Functions
    std::vector<Diagnostic::DiagnosticDefinition> check_programvariables();
    static WindowManager::ScreenCoordinatePixel convertCoordinate(
        WindowManager::ScreenCoordinatePerc coord_perc, uint16_t width_pix, uint16_t height_pix);
    std::string pretty() {
        std::string str = "";
        std::map<std::string, WindowManager>::iterator it = windows.begin();
        while (it != windows.end()) {
            str += it->second.pretty();
            ++it;
        }
        return str;
    }
    std::string get_taskheader();
    std::string get_deviceheader();
    // Printing Functions
    static std::string pretty(std::map<std::string, SystemMonitorProcess::Task> task_list) {
        std::string str = "--- Task List ---\n";
        if (task_list.size() == 0) {
            str += "\tNo Tasks Defined!\n";
            return str;
        }
        std::map<std::string, SystemMonitorProcess::Task>::iterator it = task_list.begin();
        int i = 0;
        while (it != task_list.end()) {
            char tempstr[512];
            sprintf(tempstr,
                    "\t[%d/%d] Type: %d Name: %s Rx: %4.2f\n",
                    (uint16_t)i + 1,
                    (uint16_t)task_list.size(),
                    (uint8_t)it->second.type,
                    it->second.node_name.c_str(),
                    it->second.last_heartbeat_delta);
            str += std::string(tempstr);
            i++;
            ++it;
        }
        return str;
    }
    static std::string pretty(std::map<std::string, SystemMonitorProcess::Device> device_list) {
        std::string str = "--- Device List ---\n";
        if (device_list.size() == 0) {
            str += "\tNo Devices Defined!\n";
            return str;
        }
        std::map<std::string, SystemMonitorProcess::Device>::iterator it = device_list.begin();
        int i = 0;
        while (it != device_list.end()) {
            char tempstr[512];
            sprintf(tempstr,
                    "\t[%d/%d] Name: %s\n",
                    (uint16_t)i + 1,
                    (uint16_t)device_list.size(),
                    it->second.name.c_str());
            str += std::string(tempstr);
            i++;
            ++it;
        }
        return str;
    }
    // Destructors
    bool get_killme() {
        return kill_me;
    }
    void cleanup() {
        base_cleanup();
        std::map<std::string, WindowManager>::iterator win_it = windows.begin();
        while (win_it != windows.end()) {
            delwin(win_it->second.get_window_reference());
            ++win_it;
        }
        endwin();
    }

   private:
    std::string get_task_info(Task task, bool task_selected);
    std::string get_device_info(Device device, bool device_selected);
    bool update_task_list(eros::heartbeat heartbeat) {
        std::string key = heartbeat.NodeName;
        std::map<std::string, Task>::iterator it;
        it = task_list.find(key);
        if (it == task_list.end()) {
            return false;  // Should not do anything
        }
        else {
            it->second.host_device = heartbeat.HostName;
            it->second.base_node_name = heartbeat.BaseNodeName;
            it->second.last_heartbeat_delta = 0.0;
            it->second.last_heartbeat = get_system_time();
            it->second.state = (Node::State)heartbeat.NodeState;
        }
        return true;
    }
    bool update_task_list(eros::resource resource) {
        std::string key = resource.Name;
        std::map<std::string, Task>::iterator it;
        it = task_list.find(key);
        if (it == task_list.end()) {
            return false;  // Should not do anything
        }
        else {
            uint16_t prev_pid = it->second.pid;
            if ((it->second.pid != 0) && (prev_pid != resource.PID)) {
                it->second.restart_count++;
            }
            it->second.pid = resource.PID;
            it->second.cpu_used_perc = resource.CPU_Perc;
            it->second.mem_used_perc = resource.RAM_Perc;
        }
        return true;
    }

    bool update_device_list(eros::resource resource_available) {
        std::string key = resource_available.Name;
        std::map<std::string, Device>::iterator it = device_list.find(key);
        if (it == device_list.end()) {
            return false;
        }
        else {
            it->second.cpu_av_perc = resource_available.CPU_Perc;
            it->second.ram_av_perc = resource_available.RAM_Perc;
            it->second.disk_av_perc = resource_available.DISK_Perc;
            it->second.state = Node::State::RUNNING;
            it->second.last_heartbeat_delta = 0.0;
        }
        return true;
    }
    bool update_device_list(eros::loadfactor loadfactor) {
        std::string key = loadfactor.DeviceName;
        std::map<std::string, Device>::iterator it = device_list.find(key);
        if (it == device_list.end()) {
            return false;
        }
        else {
            if (loadfactor.loadfactor.size() != 3) {
                return false;
            }
            it->second.load_factor.at(0) = loadfactor.loadfactor.at(0);
            it->second.load_factor.at(1) = loadfactor.loadfactor.at(1);
            it->second.load_factor.at(2) = loadfactor.loadfactor.at(2);
            it->second.state = Node::State::RUNNING;
            it->second.last_heartbeat_delta = 0.0;
        }
        return true;
    }

    bool kill_me;
    ros::NodeHandle* nodeHandle;
    ros::Publisher command_pub;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    std::map<TaskFieldColumn, Field> task_window_fields;
    std::map<std::string, WindowManager> windows;
    std::map<std::string, Task> task_list;
    std::map<uint16_t, std::string> task_name_list;
    std::map<std::string, std::string> resource_topics;
    std::map<DeviceFieldColumn, Field> device_window_fields;
    std::map<std::string, Device> device_list;
    std::map<uint16_t, std::string> device_name_list;
    bool select_task_mode;
    bool change_log_level_mode;
    bool show_task_diagnostic_mode;
    bool change_nodestate_mode;
    bool select_device_mode;
    int16_t selected_task_index;
    uint16_t start_node_index;
    uint16_t task_list_max_rows;
    double timer_showing_message_in_window;
    std::string message_text;
    Color message_text_color;
    std::vector<Diagnostic::DiagnosticDefinition> task_diagnostics_to_show;
    ArmDisarm::State armed_state;
};
#endif  // SYSTEMMONITORPROCESS_h