/*! \file SystemMonitor.h
 */
// No practical way to this file due to screen rendering.
// LCOV_EXCL_START
#ifndef SystemMonitor_H
#define SystemMonitor_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project
#include <eros/BaseNode.h>
#include <algorithm>
#include "SystemMonitorProcess.h"
/*! \class SystemMonitor SystemMonitor.h "SystemMonitor.h"
 *  \brief */
class SystemMonitor : public eros::BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "system_monitor";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 5;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 0;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 16-July-2021";

    /*! \brief What System this Node falls under.*/
    const eros::System::MainSystem DIAGNOSTIC_SYSTEM = eros::System::MainSystem::ROVER;

    /*! \brief What Subsystem this Node falls under.*/
    const eros::System::SubSystem DIAGNOSTIC_SUBSYSTEM = eros::System::SubSystem::ENTIRE_SYSTEM;

    /*! \brief What Component this Node falls under.*/
    const eros::System::Component DIAGNOSTIC_COMPONENT = eros::System::Component::ENTIRE_SUBSYSTEM;
    SystemMonitor();
    ~SystemMonitor();
    SystemMonitorProcess* get_process() {
        return process;
    }
    bool start();
    eros::Diagnostic::DiagnosticDefinition finish_initialization();
    bool run_loop1();
    bool run_loop2();
    bool run_loop3();
    bool run_001hz();
    bool run_01hz();
    bool run_01hz_noisy();
    bool run_1hz();
    bool run_10hz();
    void thread_loop();
    void cleanup();

    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_commandAction_Callback(const eros::system_commandGoalConstPtr& goal);
    void command_Callback(const eros::command::ConstPtr& t_msg);
    void heartbeat_Callback(const eros::heartbeat::ConstPtr& msg);
    void loadfactor_Callback(const eros::loadfactor::ConstPtr& msg);
    void resourceAvailable_Callback(const eros::resource::ConstPtr& msg);
    void resourceUsed_Callback(const eros::resource::ConstPtr& msg);

   private:
    eros::Diagnostic::DiagnosticDefinition rescan_nodes();
    eros::Diagnostic::DiagnosticDefinition read_launchparameters();
    SystemMonitorProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;

    std::map<std::string, bool> filter_list;
    std::map<std::string, ros::Subscriber> heartbeat_subs;
    std::map<std::string, ros::Subscriber> resourceavailable_subs;
    std::map<std::string, ros::Subscriber> loadfactor_subs;
    std::map<std::string, ros::Subscriber> resourceused_subs;
    std::vector<std::string> genericNodeList;
};
#endif  // SystemMonitor_H
        // LCOV_EXCL_STOP