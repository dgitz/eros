/*! \file SystemMonitorProcess.h
 */
#pragma once
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <curses.h>
#include <eros/BaseNodeProcess.h>
#include <eros/SnapshotNode/SnapshotProcess.h>
#include <eros/heartbeat.h>
#include <ros/ros.h>

#include "DeviceWindow/DeviceWindow.h"
#include "HeaderWindow/HeaderWindow.h"
#include "NodeWindow/NodeWindow.h"
#include "Window_Definitions.h"
namespace eros_nodes::SystemMonitor {

/*! \class SystemMonitorProcess SystemMonitorProcess.h "SystemMonitorProcess.h"
 *  \brief  The process utility for the SystemMonitorNode. */
class SystemMonitorProcess : public eros::BaseNodeProcess
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

    SystemMonitorProcess()
        : kill_me(false),
          nodeHandle(nullptr),
          robot_namespace("/"),
          mainwindow_width(0),
          mainwindow_height(0) {
    }
    ~SystemMonitorProcess();
    // Constants

    // Enums

    // Structs

    // Initialization Functions
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    void reset();
    bool set_nodeHandle(ros::NodeHandle* nh, std::string _robot_namespace) {
        nodeHandle = nh;
        robot_namespace = _robot_namespace;
        std::string systemcommand_topic = robot_namespace + "SystemCommand";
        command_pub = nodeHandle->advertise<eros::command>(systemcommand_topic, 1);

        return true;
    }
    bool initialize_windows();

    // Update Functions
    eros::Diagnostic::DiagnosticDefinition update(double t_dt, double t_ros_time);

    // Message Functions
    std::vector<eros::Diagnostic::DiagnosticDefinition> new_commandmsg(eros::command msg);
    eros::Diagnostic::DiagnosticDefinition new_commandstate(
        const eros::command_state::ConstPtr& t_msg) {
        eros::command_state state = convert_fromptr(t_msg);
        eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        return diag;
    }
    eros::Diagnostic::DiagnosticDefinition new_heartbeatmessage(
        const eros::heartbeat::ConstPtr& t_msg);
    eros::Diagnostic::DiagnosticDefinition new_resourceusedmessage(
        const eros::resource::ConstPtr& t_msg) {
        eros::resource msg = convert_fromptr(t_msg);
        eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        return diag;
    }
    eros::Diagnostic::DiagnosticDefinition new_resourceavailablemessage(
        const eros::resource::ConstPtr& t_msg) {
        eros::resource msg = convert_fromptr(t_msg);
        eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        return diag;
    }
    eros::Diagnostic::DiagnosticDefinition new_loadfactormessage(
        const eros::loadfactor::ConstPtr& t_msg) {
        eros::loadfactor msg = convert_fromptr(t_msg);
        eros::Diagnostic::DiagnosticDefinition diag = get_root_diagnostic();
        return diag;
    }
    // Attribute Functions
    void update_armedstate(eros::ArmDisarm::State armed_state) {
        header_window->new_msg(armed_state);
    }
    bool set_mainwindow(uint16_t t_mainwindow_width, uint16_t t_mainwindow_height) {
        mainwindow_width = t_mainwindow_width;
        mainwindow_height = t_mainwindow_height;
        if (mainwindow_width < MINWINDOW_WIDTH) {
            return false;
        }
        return true;
    }
    eros::Diagnostic::DiagnosticDefinition update_monitorlist(
        std::vector<std::string> heartbeat_list,
        std::vector<std::string>& new_heartbeat_topics_to_subscribe);

    // Support Functions
    std::vector<eros::Diagnostic::DiagnosticDefinition> check_programvariables();
    std::string pretty() {
        std::string str = "";
        return str;
    }

    // Destructors
    bool get_killme() {
        return kill_me;
    }
    void cleanup() {
        base_cleanup();
        endwin();
    }

   private:
    bool kill_me{false};
    ros::NodeHandle* nodeHandle;
    std::string robot_namespace;
    ros::Publisher command_pub;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    eros::ArmDisarm::State armed_state;

    std::vector<std::string> monitored_heartbeat_topics;

    // Windows
    IWindow* header_window;
    IWindow* device_window;
    IWindow* node_window;
};
}  // namespace eros_nodes::SystemMonitor
