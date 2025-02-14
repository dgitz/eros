#pragma once
#include <mutex>

#include "BaseWindow.h"
namespace eros_nodes::SystemMonitor {
enum class NodeType { UNKNOWN = 0, EROS = 1, NON_EROS = 2 };
enum class NodeFieldColumn {
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
/*! \struct NodeData
\brief NodeData container, used for holding Node parameters.
*/
struct NodeData {
    NodeData(int16_t _id,
             NodeType _type,
             std::string _host_device,
             std::string _base_node_name,
             std::string _node_name)
        : id(_id),
          state(eros::Node::State::START),
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
    eros::Node::State state;
    NodeType type;
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
class NodeWindow : public BaseWindow
{
   public:
    static constexpr double START_X_PERC =
        0.0; /*!< What percentage of the screen to put top left corner (X) of window. */
    static constexpr double START_Y_PERC =
        15.0; /*!< What percentage of the screen to put top left corner (Y) of window. */
    static constexpr double WIDTH_PERC =
        66.0; /*!< What percentage of the screen (Width) to draw the window. */
    static constexpr double HEIGHT_PERC =
        60.0; /*!< What percentage of the screen (Height) to draw the window. */
    NodeWindow(ros::NodeHandle* nodeHandle,
               std::string robot_namespace,
               eros::Logger* logger,
               int16_t tab_order,
               uint16_t mainwindow_height,
               uint16_t mainwindow_width)
        : BaseWindow("node_window",
                     tab_order,
                     START_X_PERC,
                     START_Y_PERC,
                     WIDTH_PERC,
                     HEIGHT_PERC,
                     nodeHandle,
                     robot_namespace,
                     logger,
                     mainwindow_height,
                     mainwindow_width) {
        supported_keys.push_back(KEY_UP);
        supported_keys.push_back(KEY_DOWN);
        supported_keys.push_back(KEY_f);
        supported_keys.push_back(KEY_F);
        supported_keys.push_back(KEY_l);
        supported_keys.push_back(KEY_L);
        supported_keys.push_back(KEY_n);
        supported_keys.push_back(KEY_N);
        supported_keys.push_back(KEY_d);
        supported_keys.push_back(KEY_D);
        supported_keys.push_back(KEY_1);
        supported_keys.push_back(KEY_2);
        supported_keys.push_back(KEY_3);
        supported_keys.push_back(KEY_4);
        supported_keys.push_back(KEY_5);
        supported_keys.push_back(KEY_6);
        supported_keys.push_back(KEY_7);
        supported_keys.push_back(KEY_8);
        supported_keys.push_back(KEY_9);
        // Don't add Supported Keys for Numeric
        set_window_records_are_selectable(true);
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::MARKER, Field("", 3)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::ID, Field("ID", 4)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::HOSTNAME, Field(" Host ", 20)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::NODENAME, Field(" NodeName ", 30)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::STATUS, Field(" Status ", 10)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::RESTARTS, Field(" Restarts ", 10)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::PID, Field(" PID ", 8)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::CPU, Field(" CPU(%) ", 10)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::RAM, Field(" RAM(%) ", 10)));
        node_window_fields.insert(
            std::pair<NodeFieldColumn, Field>(NodeFieldColumn::RX, Field(" Rx ", 6)));
        ScreenCoordinatePixel coord_pix = SystemMonitorUtility::convertCoordinate(
            get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = SystemMonitorUtility::create_newwin(coord_pix.height_pix,
                                                          coord_pix.width_pix,
                                                          coord_pix.start_y_pix,
                                                          coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);

        std::string header = get_nodeheader();
        mvwprintw(win, 1, 1, header.c_str());
        std::string dashed(get_screen_coordinates_pixel().width_pix - 2, '-');
        mvwprintw(win, 2, 1, dashed.c_str());
        wrefresh(win);
    }
    virtual ~NodeWindow();
    bool is_selectable() override {
        return true;
    }
    bool update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State /* armed_state */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::loadfactor /*loadfactor_msg*/) override {  // Not Used
        return true;
    }
    bool new_msg(eros::command_state /* command_state_msg */) override {  // Not Used
        return true;
    }
    bool new_msg(eros::heartbeat heartbeat_msg) override;
    bool new_msg(eros::resource resource_used_msg) override;
    KeyEventContainer new_keyevent(int key) override;
    std::string get_node_info(NodeData node, bool selected);
    bool new_command(std::vector<WindowCommand> /* commands*/) override {  // Not Used
        return true;
    }
    std::vector<std::string> get_node_info();

   private:
    bool insertNode(NodeType node_type,
                    std::string device,
                    std::string base_node_name,
                    std::string node_name);
    std::string get_nodeheader();
    bool update_window();
    int previous_key{-1};
    std::mutex node_list_mutex;
    std::map<NodeFieldColumn, Field> node_window_fields;
    std::vector<NodeData> node_list;
};
}  // namespace eros_nodes::SystemMonitor