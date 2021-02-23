/*! \file systemmonitor_node.h
 */
#ifndef SYSTEMMONITORNODE_H
#define SYSTEMMONITORNODE_H
// C System Files
// C++ System Files
// ROS Base Functionality
// ROS Messages
// Project

#include <eros/BaseNode.h>

#include "SystemMonitorProcess.h"

/*! \class SystemMonitorNode SystemMonitorNode.h "SystemMonitorNode.h"
 *  \brief */
class SystemMonitorNode : public BaseNode
{
   public:
    /*! \brief The base name of the Node.*/
    const std::string BASE_NODE_NAME = "system_monitor";

    /*! \brief The Major Release Version of the Node.*/
    const uint16_t MAJOR_RELEASE_VERSION = 0;

    /*! \brief The Minor Release Version of the Node.*/
    const uint16_t MINOR_RELEASE_VERSION = 2;

    /*! \brief The Build Number of the Node.*/
    const uint16_t BUILD_NUMBER = 1;

    /*! \brief A Description of the Firmware.*/
    const std::string FIRMWARE_DESCRIPTION = "Latest Rev: 22-Feb-2021";

    /*! \brief What System this Node falls under.*/
    const System::MainSystem DIAGNOSTIC_SYSTEM = System::MainSystem::REMOTE_CONTROL;

    /*! \brief What Subsystem this Node falls under.*/
    const System::SubSystem DIAGNOSTIC_SUBSYSTEM = System::SubSystem::ROBOT_MONITOR;

    /*! \brief What Component this Node falls under.*/
    const System::Component DIAGNOSTIC_COMPONENT = System::Component::DIAGNOSTIC;
    SystemMonitorNode();
    ~SystemMonitorNode();
    SystemMonitorProcess* get_process() {
        return process;
    }
    bool start();
    bool init_screen();
    Diagnostic::DiagnosticDefinition finish_initialization();
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

    Diagnostic::DiagnosticDefinition rescan_nodes();
    void heartbeat_Callback(const eros::heartbeat::ConstPtr& msg);
    void resourceused_Callback(const eros::resource::ConstPtr& msg);
    void resourceavailable_Callback(const eros::resource::ConstPtr& msg);
    void loadfactor_Callback(const eros::loadfactor::ConstPtr& msg);
    bool changenodestate_service(eros::srv_change_nodestate::Request& req,
                                 eros::srv_change_nodestate::Response& res);
    void system_command_Callback(const eros::system_commandGoalConstPtr& goal);

   private:
    std::vector<ros::Subscriber> heartbeat_subs;
    std::vector<ros::Subscriber> resource_used_subs;
    std::vector<ros::Subscriber> resource_available_subs;
    std::vector<ros::Subscriber> loadfactor_subs;
    std::map<std::string, bool> filter_list;
    Diagnostic::DiagnosticDefinition read_launchparameters();
    SystemMonitorProcess* process;
    actionlib::SimpleActionServer<eros::system_commandAction> system_command_action_server;
};

#endif  // SYSTEMMONITORNODE_H
