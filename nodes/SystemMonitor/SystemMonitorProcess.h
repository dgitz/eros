/*! \file SystemMonitorProcess.h
 */
#ifndef SYSTEMMONITORPROCESS_h
#define SYSTEMMONITORPROCESS_h
#include <curses.h>
#include <eros/BaseNodeProcess.h>
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
    const double TIME_TO_SHOW_MESSAGES = 5.0f;  // Seconds

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

    enum class Color {
        UNKNOWN = 0,
        NO_COLOR = 1,
        RED_COLOR = 2,
        YELLOW_COLOR = 3,
        GREEN_COLOR = 4,
        BLUE_COLOR = 5,
        END_OF_LIST = 6
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
              cpu_used_perc(0),
              mem_used_perc(0),
              last_heartbeat(0.0),
              last_heartbeat_delta(0.0),
              restart_count(0) {
        }
        bool initialized;
        uint16_t id;
        Node::State state;
        TaskType type;
        int32_t pid;
        std::string host_device;
        std::string base_node_name;
        std::string node_name;
        int16_t cpu_used_perc;
        int32_t mem_used_perc;
        double last_heartbeat;
        double last_heartbeat_delta;
        uint64_t restart_count;
    };

    struct TaskField {
        TaskField(std::string _text, uint16_t _width) : text(_text), width(_width) {
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
          selected_task_index(-1),
          start_node_index(0),
          task_list_max_rows(5),
          timer_showing_message_in_window(0.0),
          message_text("") {
        task_window_fields.insert(
            std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::MARKER, TaskField("", 3)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::ID, TaskField("ID", 4)));
        task_window_fields.insert(std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::HOSTNAME,
                                                                        TaskField(" Host ", 20)));
        task_window_fields.insert(std::pair<TaskFieldColumn, TaskField>(
            TaskFieldColumn::NODENAME, TaskField(" NodeName ", 30)));
        task_window_fields.insert(std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::STATUS,
                                                                        TaskField(" Status ", 10)));
        task_window_fields.insert(std::pair<TaskFieldColumn, TaskField>(
            TaskFieldColumn::RESTARTS, TaskField(" Restarts ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::PID, TaskField(" PID ", 8)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::CPU, TaskField(" CPU(%) ", 10)));
        task_window_fields.insert(std::pair<TaskFieldColumn, TaskField>(
            TaskFieldColumn::RAM, TaskField(" RAM(Mb) ", 10)));
        task_window_fields.insert(
            std::pair<TaskFieldColumn, TaskField>(TaskFieldColumn::RX, TaskField(" Rx ", 6)));
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
        return true;
    }

    // Update Functions
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    Diagnostic::DiagnosticDefinition update_taskwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_instructionwindow(
        std::map<std::string, WindowManager>::iterator it);
    Diagnostic::DiagnosticDefinition update_messagewindow(
        std::map<std::string, WindowManager>::iterator it);

    Diagnostic::DiagnosticDefinition update_nodelist(
        std::vector<std::string> node_list,
        std::vector<std::string> heartbeat_list,
        std::vector<std::string>& new_heartbeat_topics_to_subscribe);
    std::map<std::string, Task> get_task_list() {
        return task_list;
    }

    // Message Functions
    std::vector<Diagnostic::DiagnosticDefinition> new_commandmsg(
        const eros::command::ConstPtr& t_msg);
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
    void set_message_text(std::string text) {
        message_text = text;
        timer_showing_message_in_window = 0.0;
    }

    // Attribute Functions
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
        }
        return true;
    }
    bool kill_me;
    ros::NodeHandle* nodeHandle;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    std::map<TaskFieldColumn, TaskField> task_window_fields;
    std::map<std::string, WindowManager> windows;
    std::map<std::string, Task> task_list;
    std::map<uint16_t, std::string> task_name_list;
    std::map<std::string, std::string> resource_topics;
    bool select_task_mode;
    int16_t selected_task_index;
    uint16_t start_node_index;
    uint16_t task_list_max_rows;
    double timer_showing_message_in_window;
    std::string message_text;
};
#endif  // SYSTEMMONITORPROCESS_h