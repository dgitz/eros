/*! \file SystemMonitorProcess.h
 */
#ifndef SYSTEMMONITORPROCESS_h
#define SYSTEMMONITORPROCESS_h
#include <curses.h>
#include <eros/BaseNodeProcess.h>
#include <eros/heartbeat.h>
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
          window_reference(nullptr) {
    }
    ~WindowManager() {
    }
    std::string get_name() {
        return name;
    }
    ScreenCoordinatePerc get_screen_coordinates_perc() {
        return screen_coord_perc;
    }
    std::string pretty() {
        std::string str = "\nWindow: " + name + "\n";
        str += "\tStart X: " + std::to_string(screen_coord_perc.start_x_perc) +
               " Start Y: " + std::to_string(screen_coord_perc.start_y_perc) + "\n";
        str += "\tWidth: " + std::to_string(screen_coord_perc.width_perc) +
               " Height: " + std::to_string(screen_coord_perc.height_perc) + "\n";
        if (window_reference == nullptr) {
            str += "\tWINDOW is Uninitialized Still.";
        }
        return str;
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
    WINDOW* window_reference;
};
/*! \class SystemMonitorProcess SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief  */
class SystemMonitorProcess : public BaseNodeProcess
{
   public:
    /*! \brief How long in seconds before marking a Node as Timed Out.*/
    const double COMMTIMEOUT_THRESHOLD = 5.0f;
    /*! \brief The minimum width in pixels of the Main Window.*/
    const uint16_t MINWINDOW_WIDTH = 140;
    /*! \brief The minimum height in pixels of the Main Window.*/
    const uint16_t MINWINDOW_HEIGHT = 240;

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
    SystemMonitorProcess() : mainwindow_width(0), mainwindow_height(0) {
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

    // Update Functions
    Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);
    Diagnostic::DiagnosticDefinition update_taskwindow(
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

    // Destructors
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
    std::string get_task_info(Task task);
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
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    std::map<TaskFieldColumn, TaskField> task_window_fields;
    std::map<std::string, WindowManager> windows;
    std::map<std::string, Task> task_list;
    std::map<std::string, std::string> resource_topics;
};
#endif  // SYSTEMMONITORPROCESS_h