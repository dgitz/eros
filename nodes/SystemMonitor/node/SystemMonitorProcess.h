/*! \file SystemMonitorProcess.h
 */
#pragma once
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <curses.h>
#include <eros/BaseNodeProcess.h>
#include <eros/heartbeat.h>
#include <ros/ros.h>

#include "DeviceWindow/DeviceWindow.h"
#include "DiagnosticsWindow/DiagnosticsWindow.h"
#include "HeaderWindow/HeaderWindow.h"
#include "InstructionWindow/InstructionWindow.h"
#include "MessageWindow/MessageWindow.h"
#include "NodeWindow/NodeWindow.h"
#include "StatusWindow/StatusWindow.h"
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
    eros::eros_diagnostic::Diagnostic finish_initialization();
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
    eros::eros_diagnostic::Diagnostic update(double t_dt, double t_ros_time);

    // Message Functions
    std::vector<eros::eros_diagnostic::Diagnostic> new_commandmsg(eros::command msg);
    eros::eros_diagnostic::Diagnostic new_commandstate(const eros::command_state::ConstPtr& t_msg);
    eros::eros_diagnostic::Diagnostic new_heartbeatmessage(const eros::heartbeat::ConstPtr& t_msg);
    eros::eros_diagnostic::Diagnostic new_resourceusedmessage(
        const eros::resource::ConstPtr& t_msg);
    eros::eros_diagnostic::Diagnostic new_resourceavailablemessage(
        const eros::resource::ConstPtr& t_msg);
    eros::eros_diagnostic::Diagnostic new_loadfactormessage(
        const eros::loadfactor::ConstPtr& t_msg);
    // Attribute Functions
    void update_armedstate(eros::ArmDisarm::State armed_state);
    bool set_mainwindow(uint16_t t_mainwindow_width, uint16_t t_mainwindow_height) {
        mainwindow_width = t_mainwindow_width;
        mainwindow_height = t_mainwindow_height;
        if (mainwindow_width < MINWINDOW_WIDTH) {
            return false;
        }
        return true;
    }
    eros::eros_diagnostic::Diagnostic update_monitorlist(
        std::vector<std::string> heartbeat_list,
        std::vector<std::string> resourceused_list,
        std::vector<std::string> resourceavailable_list,
        std::vector<std::string> loadfactor_list,
        std::vector<std::string>& new_heartbeat_topics_to_subscribe,
        std::vector<std::string>& new_resourceused_topics_to_subscribe,
        std::vector<std::string>& new_resourceavailable_topics_to_subscribe,
        std::vector<std::string>& new_loadfactor_topics_to_subscribe);

    // Support Functions
    std::vector<eros::eros_diagnostic::Diagnostic> check_programvariables();
    std::string pretty();

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
    ros::Publisher command_pub;
    std::string robot_namespace;
    uint16_t mainwindow_width;
    uint16_t mainwindow_height;
    eros::ArmDisarm::State armed_state;

    std::vector<std::string> monitored_heartbeat_topics;
    std::vector<std::string> monitored_resourceused_topics;
    std::vector<std::string> monitored_resourceavailable_topics;
    std::vector<std::string> monitored_loadfactor_topics;

    std::vector<IWindow*> windows;
    int16_t tab_index{0};
    int16_t highest_tab_index{0};
};
}  // namespace eros_nodes::SystemMonitor
