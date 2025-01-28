#pragma once
#include "../BaseWindow.h"
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
    NodeData(uint16_t _id,
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
    NodeWindow(eros::Logger* logger, uint16_t mainwindow_height, uint16_t mainwindow_width)
        : BaseWindow(
              "node_window", 0.0, 15.0, 66.0, 60.0, logger, mainwindow_height, mainwindow_width) {
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

        logger->log_warn("Initialized Node Window");
        ScreenCoordinatePixel coord_pix =
            convertCoordinate(get_screen_coordinates_perc(), mainwindow_width, mainwindow_height);
        WINDOW* win = create_newwin(coord_pix.height_pix,
                                    coord_pix.width_pix,
                                    coord_pix.start_y_pix,
                                    coord_pix.start_x_pix);
        set_screen_coordinates_pix(coord_pix);
        set_window(win);
        wrefresh(win);
    }
    virtual ~NodeWindow();
    eros::Diagnostic::DiagnosticDefinition update(double dt, double t_ros_time) override;
    bool new_msg(eros::ArmDisarm::State armed_state) override;
    bool new_msg(eros::heartbeat heartbeat_msg);
    std::string get_node_info(NodeData node, bool selected);

   private:
    std::map<NodeFieldColumn, Field> node_window_fields;
    std::map<std::string, NodeData> node_list;
};
}  // namespace eros_nodes::SystemMonitor